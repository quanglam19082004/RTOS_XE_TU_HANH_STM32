/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    mpu6050.c
  * @brief   MPU6050 driver using software I2C on PB8/PB9; computes roll, pitch
  *          (from accel+gyro complementary filter) and yaw by integrating gyro z-rate.
  *
  * FIXES từ version cũ:
  *  1. Complementary filter alpha chuyển sang time-based (dùng tau) thay vì
  *     hằng số cứng 0.98 — tránh sai tỷ lệ khi dt thay đổi.
  *  2. Sửa bug double-subtract trong auto bias correction.
  *  3. Bỏ LP filter trên gz trước khi tích phân yaw — filter làm giảm biên độ
  *     khi xoay nhanh khiến đọc 90° chỉ thấy ~60°.
  *  4. Giảm GYRO_DEADZONE và chỉ áp dụng cho yaw (không cắt roll/pitch).
  *  5. Clamp dt chặt hơn và lưu last_tick SAU khi đọc xong để dt phản ánh
  *     đúng chu kỳ gọi hàm.
  ******************************************************************************
  */
/* USER CODE END Header */

#include "mpu6050.h"
#include "stm32f4xx_hal.h"
#include <math.h>
#include <stdbool.h>

/* ── Register map ─────────────────────────────────────────────────────────── */
#define MPU6050_REG_PWR_MGMT_1    0x6B
#define MPU6050_REG_PWR_MGMT_2    0x6C
#define MPU6050_REG_SMPLRT_DIV    0x19
#define MPU6050_REG_CONFIG        0x1A
#define MPU6050_REG_ACCEL_XOUT_H  0x3B
#define MPU6050_REG_GYRO_XOUT_H   0x43
#define MPU6050_REG_GYRO_CONFIG   0x1B
#define MPU6050_REG_ACCEL_CONFIG  0x1C

/* ── MPU6050 configuration ────────────────────────────────────────────────── */
#define MPU6050_CLOCK_PLL_XGYRO   0x01
#define MPU6050_DLPF_44HZ         0x03
#define MPU6050_SMPLRT_DIV_125HZ  0x07

/* ── Software I2C pins ────────────────────────────────────────────────────── */
#define MPU6050_SCL_PIN           GPIO_PIN_8
#define MPU6050_SDA_PIN           GPIO_PIN_9
#define MPU6050_GPIO_PORT         GPIOB

/* ── Sensor scale factors ─────────────────────────────────────────────────── */
static const float MPU6050_ACCEL_SCALE = 16384.0f;  /* ±2g  → LSB/g  */
static const float MPU6050_GYRO_SCALE  = 131.0f;    /* ±250°/s → LSB/(°/s) */
static const float RAD_TO_DEG         = 57.29577951308232f;

/* ── Debug exports ────────────────────────────────────────────────────────── */
volatile int16_t mpu_raw_ax = 0, mpu_raw_ay = 0, mpu_raw_az = 0;
volatile int16_t mpu_raw_gx = 0, mpu_raw_gy = 0, mpu_raw_gz = 0;
volatile float   mpu_acc_x  = 0.0f, mpu_acc_y = 0.0f, mpu_acc_z = 0.0f;
volatile float   mpu_gyro_x = 0.0f, mpu_gyro_y = 0.0f, mpu_gyro_z = 0.0f;

/* ── Axis remapping ───────────────────────────────────────────────────────── */
static int map_swapXY  = 0;
static int map_invertX = 0;
static int map_invertY = 0;
static int map_invertZ = 0;

/* ── Attitude state ───────────────────────────────────────────────────────── */
static float roll_angle  = 0.0f;
static float pitch_angle = 0.0f;
static float yaw_angle   = 0.0f;
static uint32_t last_tick = 0;

/* ── Gyro calibration ─────────────────────────────────────────────────────── */
/*
 * Hardcode offset đo được từ live expression khi sensor đứng yên:
 *   mpu_raw_gx ≈ 275  → offset =  275 / 131 ≈ +2.099 °/s
 *   mpu_raw_gy ≈ 275  → offset =  275 / 131 ≈ +2.099 °/s
 *   mpu_raw_gz ≈ -78  → offset = -78  / 131 ≈ -0.595 °/s
 *
 * Nếu sau khi flash vẫn còn sai thì đọc lại raw khi đứng yên và cập nhật
 * 3 con số dưới đây (raw_value / 131.0f).
 */
static float gyro_offset_x   =  275.0f / 131.0f;   /* ~+2.099 °/s */
static float gyro_offset_y   =  275.0f / 131.0f;   /* ~+2.099 °/s */
static float gyro_offset_z   =  -78.0f / 131.0f;   /* ~-0.595 °/s */
static int   calibration_done = 1;  /* bỏ qua auto calibration, dùng hardcode */

/* ── Filter / algorithm constants ────────────────────────────────────────── */

/*
 * Complementary filter time constant (seconds).
 * alpha = tau / (tau + dt)  — tính lại mỗi lần gọi nên đúng bất kể dt thay đổi.
 * tau = 0.5s → ở 100Hz alpha ≈ 0.981; ở 50Hz alpha ≈ 0.962.
 * Tăng tau → tin gyro hơn (ít rung nhưng drift nhiều hơn).
 * Giảm tau → tin accel hơn (ổn định hơn nhưng rung theo gia tốc tuyến tính).
 */
#define COMP_FILTER_TAU        2.0f

/*
 * Auto bias correction: chỉ chạy khi sensor gần như đứng yên.
 * Điều kiện: accel magnitude ≈ 1g và tất cả gyro rate < STATIONARY_GYRO_THRESH.
 */
#define AUTO_BIAS_ALPHA        0.002f   /* tốc độ cập nhật offset (nhỏ = chậm nhưng ổn) */
#define STATIONARY_GYRO_THRESH 0.5f    /* deg/s — ngưỡng coi là đứng yên */
#define STATIONARY_ACCEL_TOL   0.05f   /* g — độ lệch cho phép khỏi 1g */

/*
 * Yaw deadzone: chỉ dùng để tránh drift khi thực sự đứng yên.
 * Giá trị nhỏ để không cắt mất chuyển động chậm.
 */
#define YAW_DEADZONE_DEGS      0.3f    /* deg/s */

#define GYRO_CAL_SAMPLES       200

/* ══════════════════════════════════════════════════════════════════════════
   Software I2C implementation
   ══════════════════════════════════════════════════════════════════════════ */

static void MPU6050_I2C_Delay(void)
{
    for (volatile int i = 0; i < 100; ++i) __NOP();
}

static void MPU6050_I2C_PinInit(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    __HAL_RCC_GPIOB_CLK_ENABLE();
    GPIO_InitStruct.Pin   = MPU6050_SCL_PIN | MPU6050_SDA_PIN;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull  = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(MPU6050_GPIO_PORT, &GPIO_InitStruct);
    HAL_GPIO_WritePin(MPU6050_GPIO_PORT, MPU6050_SCL_PIN | MPU6050_SDA_PIN, GPIO_PIN_SET);
}

static void MPU6050_I2C_SCL_Set(GPIO_PinState state)
{
    HAL_GPIO_WritePin(MPU6050_GPIO_PORT, MPU6050_SCL_PIN, state);
    MPU6050_I2C_Delay();
}

static void MPU6050_I2C_SDA_Set(GPIO_PinState state)
{
    HAL_GPIO_WritePin(MPU6050_GPIO_PORT, MPU6050_SDA_PIN, state);
    MPU6050_I2C_Delay();
}

static GPIO_PinState MPU6050_I2C_SDA_Read(void)
{
    return HAL_GPIO_ReadPin(MPU6050_GPIO_PORT, MPU6050_SDA_PIN);
}

static void MPU6050_I2C_SDA_OutputMode(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin   = MPU6050_SDA_PIN;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull  = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(MPU6050_GPIO_PORT, &GPIO_InitStruct);
}

static void MPU6050_I2C_SDA_InputMode(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin  = MPU6050_SDA_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(MPU6050_GPIO_PORT, &GPIO_InitStruct);
}

static void MPU6050_I2C_Start(void)
{
    MPU6050_I2C_SDA_OutputMode();
    MPU6050_I2C_SDA_Set(GPIO_PIN_SET);
    MPU6050_I2C_SCL_Set(GPIO_PIN_SET);
    MPU6050_I2C_SDA_Set(GPIO_PIN_RESET);
    MPU6050_I2C_SCL_Set(GPIO_PIN_RESET);
}

static void MPU6050_I2C_Stop(void)
{
    MPU6050_I2C_SDA_OutputMode();
    MPU6050_I2C_SDA_Set(GPIO_PIN_RESET);
    MPU6050_I2C_SCL_Set(GPIO_PIN_SET);
    MPU6050_I2C_SDA_Set(GPIO_PIN_SET);
}

static bool MPU6050_I2C_WriteByte(uint8_t data)
{
    for (uint8_t i = 0; i < 8; ++i) {
        MPU6050_I2C_SDA_Set((data & 0x80) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        MPU6050_I2C_SCL_Set(GPIO_PIN_SET);
        MPU6050_I2C_SCL_Set(GPIO_PIN_RESET);
        data <<= 1;
    }
    /* Read ACK */
    MPU6050_I2C_SDA_Set(GPIO_PIN_SET);
    MPU6050_I2C_SDA_InputMode();
    MPU6050_I2C_SCL_Set(GPIO_PIN_SET);
    bool ack = (MPU6050_I2C_SDA_Read() == GPIO_PIN_RESET);
    MPU6050_I2C_SCL_Set(GPIO_PIN_RESET);
    MPU6050_I2C_SDA_OutputMode();
    return ack;
}

static uint8_t MPU6050_I2C_ReadByte(bool ack)
{
    uint8_t data = 0;
    MPU6050_I2C_SDA_Set(GPIO_PIN_SET);
    MPU6050_I2C_SDA_InputMode();
    for (uint8_t i = 0; i < 8; ++i) {
        data <<= 1;
        MPU6050_I2C_SCL_Set(GPIO_PIN_SET);
        if (MPU6050_I2C_SDA_Read() == GPIO_PIN_SET) data |= 0x01;
        MPU6050_I2C_SCL_Set(GPIO_PIN_RESET);
    }
    MPU6050_I2C_SDA_OutputMode();
    MPU6050_I2C_SDA_Set(ack ? GPIO_PIN_RESET : GPIO_PIN_SET);
    MPU6050_I2C_SCL_Set(GPIO_PIN_SET);
    MPU6050_I2C_SCL_Set(GPIO_PIN_RESET);
    MPU6050_I2C_SDA_Set(GPIO_PIN_SET);
    return data;
}

static bool MPU6050_I2C_WriteRegs(uint8_t reg, const uint8_t *data, uint16_t size)
{
    MPU6050_I2C_Start();
    if (!MPU6050_I2C_WriteByte(MPU6050_I2C_ADDRESS | 0x00)) { MPU6050_I2C_Stop(); return false; }
    if (!MPU6050_I2C_WriteByte(reg))                         { MPU6050_I2C_Stop(); return false; }
    for (uint16_t i = 0; i < size; ++i) {
        if (!MPU6050_I2C_WriteByte(data[i]))                 { MPU6050_I2C_Stop(); return false; }
    }
    MPU6050_I2C_Stop();
    return true;
}

static bool MPU6050_I2C_ReadRegs(uint8_t reg, uint8_t *buffer, uint16_t size)
{
    MPU6050_I2C_Start();
    if (!MPU6050_I2C_WriteByte(MPU6050_I2C_ADDRESS | 0x00)) { MPU6050_I2C_Stop(); return false; }
    if (!MPU6050_I2C_WriteByte(reg))                         { MPU6050_I2C_Stop(); return false; }
    MPU6050_I2C_Start();
    if (!MPU6050_I2C_WriteByte(MPU6050_I2C_ADDRESS | 0x01)) { MPU6050_I2C_Stop(); return false; }
    for (uint16_t i = 0; i < size; ++i) {
        buffer[i] = MPU6050_I2C_ReadByte(i < (size - 1));
    }
    MPU6050_I2C_Stop();
    return true;
}

/* ══════════════════════════════════════════════════════════════════════════
   Axis remapping
   ══════════════════════════════════════════════════════════════════════════ */

void MPU6050_SetAxisMapping(int swapXY, int invertX, int invertY, int invertZ)
{
    map_swapXY  = swapXY  ? 1 : 0;
    map_invertX = invertX ? 1 : 0;
    map_invertY = invertY ? 1 : 0;
    map_invertZ = invertZ ? 1 : 0;
}

/* ══════════════════════════════════════════════════════════════════════════
   Gyro calibration (static offset)
   ══════════════════════════════════════════════════════════════════════════ */

static void MPU6050_CalibrateGyro(void)
{
    if (calibration_done) return;

    int32_t sumX = 0, sumY = 0, sumZ = 0;
    int     count = 0;

    for (int i = 0; i < GYRO_CAL_SAMPLES; ++i) {
        uint8_t rawData[14];
        if (MPU6050_I2C_ReadRegs(MPU6050_REG_ACCEL_XOUT_H, rawData, 14)) {
            sumX += (int16_t)((rawData[8]  << 8) | rawData[9]);
            sumY += (int16_t)((rawData[10] << 8) | rawData[11]);
            sumZ += (int16_t)((rawData[12] << 8) | rawData[13]);
            ++count;
        }
        HAL_Delay(5);   /* 5ms × 200 = 1s calibration window */
    }

    if (count > 0) {
        gyro_offset_x = (float)sumX / (count * MPU6050_GYRO_SCALE);
        gyro_offset_y = (float)sumY / (count * MPU6050_GYRO_SCALE);
        gyro_offset_z = (float)sumZ / (count * MPU6050_GYRO_SCALE);
    }
    calibration_done = 1;
}

/* ══════════════════════════════════════════════════════════════════════════
   Public API
   ══════════════════════════════════════════════════════════════════════════ */

bool MPU6050_Init(void)
{
    MPU6050_I2C_PinInit();
    uint8_t data;

    /* Wake up, use X-gyro PLL as clock source */
    data = MPU6050_CLOCK_PLL_XGYRO;
    if (!MPU6050_I2C_WriteRegs(MPU6050_REG_PWR_MGMT_1, &data, 1)) return false;
    HAL_Delay(100);

    /* 125 Hz output data rate */
    data = MPU6050_SMPLRT_DIV_125HZ;
    if (!MPU6050_I2C_WriteRegs(MPU6050_REG_SMPLRT_DIV, &data, 1)) return false;

    /* 44 Hz DLPF */
    data = MPU6050_DLPF_44HZ;
    if (!MPU6050_I2C_WriteRegs(MPU6050_REG_CONFIG, &data, 1)) return false;

    /* ±2g accel, ±250°/s gyro */
    data = 0x00;
    if (!MPU6050_I2C_WriteRegs(MPU6050_REG_ACCEL_CONFIG, &data, 1)) return false;
    if (!MPU6050_I2C_WriteRegs(MPU6050_REG_GYRO_CONFIG,  &data, 1)) return false;

    /* Enable all axes */
    if (!MPU6050_I2C_WriteRegs(MPU6050_REG_PWR_MGMT_2, &data, 1)) return false;
    HAL_Delay(50);

    /* Reset attitude state */
    yaw_angle   = 0.0f;
    roll_angle  = 0.0f;
    pitch_angle = 0.0f;

    /* Static gyro calibration — sensor phải đứng yên trong lúc này */
    MPU6050_CalibrateGyro();

    /* Khởi tạo roll/pitch từ accel để tránh transient lớn khi mới bật */
    uint8_t rawData[14];
    if (MPU6050_I2C_ReadRegs(MPU6050_REG_ACCEL_XOUT_H, rawData, 14)) {
        float accX = (int16_t)((rawData[0] << 8) | rawData[1]) / MPU6050_ACCEL_SCALE;
        float accY = (int16_t)((rawData[2] << 8) | rawData[3]) / MPU6050_ACCEL_SCALE;
        float accZ = (int16_t)((rawData[4] << 8) | rawData[5]) / MPU6050_ACCEL_SCALE;

        if (map_invertX) accX = -accX;
        if (map_invertY) accY = -accY;
        if (map_invertZ) accZ = -accZ;
        if (map_swapXY)  { float t = accX; accX = accY; accY = t; }

        roll_angle  = atan2f(accY, accZ) * RAD_TO_DEG;
        pitch_angle = atan2f(-accX, sqrtf(accY * accY + accZ * accZ)) * RAD_TO_DEG;
    }

    last_tick = HAL_GetTick();
    return true;
}

bool MPU6050_ReadAngles(MPU6050_Angle_t *angle)
{
    if (angle == NULL) return false;

    /* ── 1. Đọc raw data ─────────────────────────────────────────────────── */
    uint8_t rawData[14];
    if (!MPU6050_I2C_ReadRegs(MPU6050_REG_ACCEL_XOUT_H, rawData, 14)) return false;

    int16_t rawAx = (int16_t)((rawData[0]  << 8) | rawData[1]);
    int16_t rawAy = (int16_t)((rawData[2]  << 8) | rawData[3]);
    int16_t rawAz = (int16_t)((rawData[4]  << 8) | rawData[5]);
    /* rawData[6..7] = temperature, bỏ qua */
    int16_t rawGx = (int16_t)((rawData[8]  << 8) | rawData[9]);
    int16_t rawGy = (int16_t)((rawData[10] << 8) | rawData[11]);
    int16_t rawGz = (int16_t)((rawData[12] << 8) | rawData[13]);

    /* Debug exports */
    mpu_raw_ax = rawAx; mpu_raw_ay = rawAy; mpu_raw_az = rawAz;
    mpu_raw_gx = rawGx; mpu_raw_gy = rawGy; mpu_raw_gz = rawGz;

    /* ── 2. Convert + axis remap ─────────────────────────────────────────── */
    float accX = (float)rawAx / MPU6050_ACCEL_SCALE;
    float accY = (float)rawAy / MPU6050_ACCEL_SCALE;
    float accZ = (float)rawAz / MPU6050_ACCEL_SCALE;
    float gx   = (float)rawGx / MPU6050_GYRO_SCALE;   /* deg/s */
    float gy   = (float)rawGy / MPU6050_GYRO_SCALE;
    float gz   = (float)rawGz / MPU6050_GYRO_SCALE;

    if (map_invertX) { accX = -accX; gx = -gx; }
    if (map_invertY) { accY = -accY; gy = -gy; }
    if (map_invertZ) { accZ = -accZ; gz = -gz; }
    if (map_swapXY)  {
        float t;
        t = accX; accX = accY; accY = t;
        t = gx;   gx   = gy;   gy   = t;
    }

    /* Debug exports sau remap */
    mpu_acc_x  = accX; mpu_acc_y = accY; mpu_acc_z = accZ;
    mpu_gyro_x = gx;   mpu_gyro_y = gy;  mpu_gyro_z = gz;

    /* ── 3. Tính dt ──────────────────────────────────────────────────────── */
    uint32_t now = HAL_GetTick();
    float dt = (float)(now - last_tick) / 1000.0f;
    /*
     * Giới hạn dt: nếu hàm không được gọi quá lâu (ví dụ: debug breakpoint,
     * hoặc task bị block), tránh tích phân nhảy vọt.
     * Giới hạn trên 50ms (~20Hz minimum) là hợp lý với DLPF 44Hz.
     */
    if (dt <= 0.0f) dt = 0.001f;
    last_tick = now;
    if (dt > 0.1f) {
        /* dt quá lớn — task bị block lâu (vd: HCSR04 timeout, osDelay vật cản).
         * Không tích phân khoảng này vì không có dữ liệu gyro thực tế.
         * Trả về góc hiện tại, bỏ qua lần đọc này. */
        angle->roll  = roll_angle;
        angle->pitch = pitch_angle;
        angle->yaw   = yaw_angle;
        return true;
    }

    /* ── 4. Bỏ offset gyro (static calibration) ─────────────────────────── */
    float gx_corr = gx - gyro_offset_x;
    float gy_corr = gy - gyro_offset_y;
    float gz_corr = gz - gyro_offset_z;

    /* ── 5. Auto bias correction (chỉ khi đứng yên) ─────────────────────── */
    /*
     * FIX: version cũ có bug — gx_corr bị trừ thêm sau khi offset đã cập nhật,
     * làm cho giá trị dùng để tích phân không nhất quán.
     * Version mới: cập nhật offset vào biến state, KHÔNG sửa gx_corr trong lần này.
     * Lần gọi tiếp theo sẽ dùng offset mới.
     */
    float accel_mag = sqrtf(accX * accX + accY * accY + accZ * accZ);
    bool stationary = (fabsf(accel_mag - 1.0f) < STATIONARY_ACCEL_TOL)
                   && (fabsf(gx_corr) < STATIONARY_GYRO_THRESH)
                   && (fabsf(gy_corr) < STATIONARY_GYRO_THRESH)
                   && (fabsf(gz_corr) < STATIONARY_GYRO_THRESH);

    if (stationary) {
        gyro_offset_x += AUTO_BIAS_ALPHA * gx_corr;
        gyro_offset_y += AUTO_BIAS_ALPHA * gy_corr;
        gyro_offset_z += AUTO_BIAS_ALPHA * gz_corr;
        /* gx_corr/gy_corr/gz_corr KHÔNG thay đổi ở đây — dùng nguyên để tích phân */
    }

    /* ── 6. Accel-based roll/pitch (ground truth, nhạy với rung) ─────────── */
    float accel_roll  = atan2f(accY, accZ) * RAD_TO_DEG;
    float accel_pitch = atan2f(-accX, sqrtf(accY * accY + accZ * accZ)) * RAD_TO_DEG;

    /* ── 7. Complementary filter với alpha time-based ────────────────────── */
    /*
     * FIX: version cũ dùng COMPLEMENTARY_ALPHA = 0.98 cứng, chỉ đúng ở một dt nhất định.
     * Công thức alpha = tau / (tau + dt) đảm bảo tần số cắt = 1/(2π·tau) bất kể dt.
     *
     * Bước:
     *   a) Tích phân gyro vào angle hiện tại
     *   b) Fuse với accel bằng alpha time-based
     */
    float alpha = COMP_FILTER_TAU / (COMP_FILTER_TAU + dt);

    roll_angle  = alpha * (roll_angle  + gx_corr * dt) + (1.0f - alpha) * accel_roll;
    pitch_angle = alpha * (pitch_angle + gy_corr * dt) + (1.0f - alpha) * accel_pitch;

    /* ── 8. Yaw: chỉ integrate gyro (không có mag để correct) ───────────── */
    /*
     * FIX: version cũ dùng LP filter trên gz trước khi tích phân.
     * LP filter với alpha=0.2 làm giảm biên độ khi xoay nhanh → đọc 90° ra 60°.
     * Bỏ LP filter, chỉ dùng deadzone nhỏ để chặn drift khi đứng yên thực sự.
     *
     * Lưu ý: yaw vẫn drift theo thời gian vì không có magnetometer.
     * Nếu cần yaw chính xác dài hạn → cần HMC5883L/QMC5883L.
     */
    float gz_for_yaw = gz_corr;
    if (fabsf(gz_for_yaw) < YAW_DEADZONE_DEGS) {
        gz_for_yaw = 0.0f;
    }
    yaw_angle += gz_for_yaw * dt;

    /* ── 9. Output ───────────────────────────────────────────────────────── */
    angle->roll  = roll_angle;
    angle->pitch = pitch_angle;
    angle->yaw   = yaw_angle;
    return true;
}
