/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "CANSPI.h"
#include "mpu6050.h"
#include "string.h"
#include <stdio.h>
#include "math.h"
#include "stdbool.h"
#include "debug.h"
#include "odometry.h"  // <--- Code của Lâm
/* USER CODE END Includes */

/* USER CODE BEGIN Variables */
extern UART_HandleTypeDef huart1;
extern SPI_HandleTypeDef hspi2;
OdometryHandle_t hOdom; // <--- THÊM BIẾN NÀY ĐỂ LƯU TỌA ĐỘ X, Y, THETA
volatile bool  obstacle_detected    = false;
volatile uint32_t pulse_left        = 0;
volatile uint32_t pulse_right       = 0;
volatile float desired_yaw          = 0.0f;

float speed_left    = 0.0f;
float speed_right   = 0.0f;
float total_distance = 0.0f;

volatile MPU6050_Angle_t mpu6050_angle = {0.0f, 0.0f, 0.0f};
volatile float mpu6050_roll  = 0.0f;
volatile float mpu6050_pitch = 0.0f;
volatile float mpu6050_yaw   = 0.0f;

float target_velocity = 0.0f;
float target_angular  = 0.0f;
const float DISTANCE_PER_PULSE = 0.01021f;

volatile uint32_t echo_time = 0;
volatile bool     echo_done = false;

/* Turn direction tracking via MPU6050 yaw */
volatile float yaw_before_turn = 0.0f;
volatile int16_t last_turn_direction = 0;

/* Debug variables for STLink watch */
volatile uint8_t can_tx_status = 0;
volatile uint8_t can_rx_status = 0;
volatile uint32_t can_tx_count = 0;
volatile uint32_t can_rx_count = 0;
volatile uint32_t can_last_rx_id = 0;
volatile float can_last_turn_yaw = 0.0f;

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;

/* PID Controller for Yaw Heading Correction */
typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float error_integral;
    float error_prev;
    float integral_max;
    float output_max;
    float desired_yaw;
} YawPID_t;

static YawPID_t yaw_pid = {
    .Kp = 2.0f,
    .Ki = 0.3f,
    .Kd = 0.5f,
    .error_integral = 0.0f,
    .error_prev = 0.0f,
    .integral_max = 150.0f,
    .output_max = 400.0f,
    .desired_yaw = 0.0f
};

osSemaphoreId canRxSemaphoreHandle;
osMutexId spiMutexHandle;
bool heading_initialized = false;
volatile bool reset_imu_flag = false;
/* USER CODE END Variables */

osThreadId ControlTaskHandle;
osThreadId CanTXHandle;
osThreadId CanRXHandle;
osThreadId SensorTaskHandle;
osThreadId ImuTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void Servo_SetAngle(uint8_t angle);
void Motor_SetSpeed(int16_t speed_left, int16_t speed_right);
void Robot_TurnLeft(void);
void Robot_TurnRight(void);
void Robot_TurnAround(void);
float HCSR04_GetDistance(void);
static void YawPID_Reset(void);
/* USER CODE END FunctionPrototypes */

void StartControlTask(void const * argument);
void StartCanTX(void const * argument);
void StartCanRX(void const * argument);
void StartSensorTask(void const * argument);
void StartImuTask(void const * argument);

void MX_FREERTOS_Init(void);

void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer,
                                   StackType_t  **ppxIdleTaskStackBuffer,
                                   uint32_t      *pulIdleTaskStackSize);

static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t  xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer,
                                   StackType_t  **ppxIdleTaskStackBuffer,
                                   uint32_t      *pulIdleTaskStackSize)
{
    *ppxIdleTaskTCBBuffer  = &xIdleTaskTCBBuffer;
    *ppxIdleTaskStackBuffer = &xIdleStack[0];
    *pulIdleTaskStackSize   = configMINIMAL_STACK_SIZE;
}

void MX_FREERTOS_Init(void)
{
    osSemaphoreDef(canRxSem);
    canRxSemaphoreHandle = osSemaphoreCreate(osSemaphore(canRxSem), 1);
    osSemaphoreWait(canRxSemaphoreHandle, 0);
    // BỔ SUNG 2 DÒNG NÀY VÀO ĐỂ TẠO MUTEX:
        osMutexDef(spiMutex);
        spiMutexHandle = osMutexCreate(osMutex(spiMutex));

    osThreadDef(ControlTask, StartControlTask, osPriorityNormal,      0, 256);
    ControlTaskHandle = osThreadCreate(osThread(ControlTask), NULL);

    osThreadDef(CanTX,       StartCanTX,       osPriorityLow,         0, 256);
    CanTXHandle = osThreadCreate(osThread(CanTX), NULL);

    osThreadDef(CanRX,       StartCanRX,       osPriorityHigh,        0, 256);
    CanRXHandle = osThreadCreate(osThread(CanRX), NULL);

    osThreadDef(SensorTask,  StartSensorTask,  osPriorityAboveNormal, 0, 512);
    SensorTaskHandle = osThreadCreate(osThread(SensorTask), NULL);

    osThreadDef(ImuTask,     StartImuTask,     osPriorityAboveNormal, 0, 256);
    ImuTaskHandle = osThreadCreate(osThread(ImuTask), NULL);
}

void StartImuTask(void const * argument)
{
    /* USER CODE BEGIN StartImuTask */
    static uint32_t imu_debug_counter = 0;
    float yaw_offset = 0.0f;
    for (;;)
    {
        MPU6050_Angle_t angle;
        if (MPU6050_ReadAngles(&angle)) {
            mpu6050_angle = angle;
            mpu6050_roll  = angle.roll;
            mpu6050_pitch = angle.pitch;
            mpu6050_yaw   = angle.yaw;
            if (reset_imu_flag == true) {
               yaw_offset = angle.yaw;
               reset_imu_flag = false; // Chốt xong thì tắt cờ
               }
            mpu6050_yaw = angle.yaw - yaw_offset;

            imu_debug_counter++;
            if (imu_debug_counter >= 2) {
                imu_debug_counter = 0;
                debug_printf("MPU6050 -> Roll:%.2f Pitch:%.2f Yaw:%.2f [RAW Ax:%d Ay:%d Az:%d Gx:%d Gy:%d Gz:%d]\r\n",
                    (double)mpu6050_roll, (double)mpu6050_pitch, (double)mpu6050_yaw,
                    mpu_raw_ax, mpu_raw_ay, mpu_raw_az, mpu_raw_gx, mpu_raw_gy, mpu_raw_gz);
            }
        } else {
            debug_printf("MPU6050 read FAILED!\r\n");
        }
        osDelay(50);
    }
    /* USER CODE END StartImuTask */
}

static float YawPID_Update(float desired_yaw, float current_yaw, float dt)
{
    float yaw_error = desired_yaw - current_yaw;
    while (yaw_error > 180.0f)  yaw_error -= 360.0f;
    while (yaw_error < -180.0f) yaw_error += 360.0f;

    float p_term = yaw_pid.Kp * yaw_error;

    yaw_pid.error_integral += yaw_error * dt;

    if (yaw_pid.error_integral > yaw_pid.integral_max) {
        yaw_pid.error_integral = yaw_pid.integral_max;
    } else if (yaw_pid.error_integral < -yaw_pid.integral_max) {
        yaw_pid.error_integral = -yaw_pid.integral_max;
    }

    float i_term = yaw_pid.Ki * yaw_pid.error_integral;

    float d_term = 0.0f;
    if (dt > 0.001f) {
        float error_derivative = (yaw_error - yaw_pid.error_prev) / dt;
        d_term = yaw_pid.Kd * error_derivative;
        yaw_pid.error_prev = yaw_error;
    }

    float correction = p_term + i_term + d_term;

    if (correction > yaw_pid.output_max) {
        correction = yaw_pid.output_max;
    } else if (correction < -yaw_pid.output_max) {
        correction = -yaw_pid.output_max;
    }

    return correction;
}

static void YawPID_Reset(void)
{
    yaw_pid.error_integral = 0.0f;
    yaw_pid.error_prev = 0.0f;
}

/* ══════════════════════════════════════════════════════════════════════════
   ControlTask — ĐÃ FIX LOGIC KHÔNG ĐÈ GÓC YAW & ODOMETRY CHUẨN
   ══════════════════════════════════════════════════════════════════════════ */
void StartControlTask(void const * argument)
{
    /* USER CODE BEGIN StartControlTask */
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
    __HAL_TIM_MOE_ENABLE(&htim1);

    // =========================================================
    // CODE LÂM: ÉP CẤU HÌNH CHÂN ENCODER QUANG 1 DÂY
    // =========================================================
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    __HAL_RCC_GPIOB_CLK_ENABLE();

    // Bánh Trái (TIM3) - Dây PB4
    GPIO_InitStruct.Pin = GPIO_PIN_4;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    // Bánh Phải (TIM4) - Dây PB6
    GPIO_InitStruct.Pin = GPIO_PIN_6;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    HAL_TIM_Base_Start(&htim3);
    HAL_TIM_Base_Start(&htim4);

    // Khởi tạo Odometry: Bánh 3.3cm, Trục 10.6cm, 330 xung/vòng
    Odometry_Init(&hOdom, 0.033f, 0.106f, 330, NULL, NULL);

    uint32_t last_time = HAL_GetTick();
    static uint32_t debug_counter = 0;
    bool heading_initialized = false; // Phải khai báo biến này

    // BẮT BUỘC ĐỂ 2 BIẾN NÀY NGOÀI VÒNG LẶP ĐỂ NHỚ CHIỀU QUAY!
    int16_t cmd_left  = 600;
    int16_t cmd_right = 600;

    for (;;)
    {
        // 1. TÍNH dt
        uint32_t current_time = HAL_GetTick();
        float dt = (float)(current_time - last_time) / 1000.0f;
        if (dt > 0.1f) dt = 0.02f;
        if (dt < 0.001f) dt = 0.001f;
        last_time = current_time;

        // 2. ĐỌC ENCODER VÀ BÙ DẤU
        pulse_left  = __HAL_TIM_GET_COUNTER(&htim3);
        pulse_right = __HAL_TIM_GET_COUNTER(&htim4);
        __HAL_TIM_SET_COUNTER(&htim3, 0);
        __HAL_TIM_SET_COUNTER(&htim4, 0);

        float v_L_raw = ((float)pulse_left  * DISTANCE_PER_PULSE) / dt;
        float v_R_raw = ((float)pulse_right * DISTANCE_PER_PULSE) / dt;

        // Dùng cmd_left của vòng lặp TRƯỚC để biết chiều quay
        speed_left  = (cmd_left  < 0) ? -v_L_raw : v_L_raw;
        speed_right = (cmd_right < 0) ? -v_R_raw : v_R_raw;

        // 3. TÍNH TOÁN ODOMETRY (Lâm thêm vào đây)
        float v_center = (speed_left + speed_right) / 2.0f;
//        float omega = (speed_right - speed_left) / 0.106f;

//        hOdom.pose.theta += omega * dt;
//        if (hOdom.pose.theta > 3.14159f)  hOdom.pose.theta -= 6.28318f;
//        if (hOdom.pose.theta < -3.14159f) hOdom.pose.theta += 6.28318f;
        // THAY BẰNG DÒNG NÀY: Ép góc Theta bằng chính góc Yaw của MPU6050 (Đổi từ Độ sang Radian)
        hOdom.pose.theta = mpu6050_yaw * (3.14159f / 180.0f);
        hOdom.pose.x += v_center * cosf(hOdom.pose.theta) * dt;
        hOdom.pose.y += v_center * sinf(hOdom.pose.theta) * dt;

        // 4. CHẠY PID CỦA NHẬT
        if (obstacle_detected == false) {
            /* Chỉ lấy mpu6050_yaw gốc đúng 1 lần duy nhất lúc mới bật nguồn */
            if (!heading_initialized) {
                desired_yaw = mpu6050_yaw;
                yaw_pid.desired_yaw = desired_yaw;
                YawPID_Reset();
                heading_initialized = true;
            }

            /* Chạy PID ổn định hướng */
            float correction = YawPID_Update(desired_yaw, mpu6050_yaw, dt);

            debug_counter++;
            if (debug_counter >= 25) {
                debug_counter = 0;
                float yaw_err = desired_yaw - mpu6050_yaw;
                while (yaw_err > 180.0f) yaw_err -= 360.0f;
                while (yaw_err < -180.0f) yaw_err += 360.0f;

                debug_printf("[PID] Desired:%.2f Current:%.2f Error:%.2f | Correction:%.1f | Cmd_L:%d Cmd_R:%d\r\n",
                    (double)desired_yaw, (double)mpu6050_yaw, (double)yaw_err, (double)correction, cmd_left, cmd_right);
            }

            // Ghi đè biến cmd_left, cmd_right ĐỂ VÒNG LẶP SAU DÙNG BÙ DẤU
            cmd_left  = (int16_t)(600.0f + correction);
            cmd_right = (int16_t)(600.0f - correction);

            if (cmd_left > 999)  cmd_left = 999;
            if (cmd_left < 0)    cmd_left = 0;
            if (cmd_right > 999) cmd_right = 999;
            if (cmd_right < 0)   cmd_right = 0;

            Motor_SetSpeed(cmd_left, cmd_right);

        } else {
            /* FIX: Có vật cản -> Tắt MPU PID, dừng xích động cơ lập tức */
            speed_left  = 0.0f;
            speed_right = 0.0f;
            // Ép cmd về 0 để vòng lặp sau Encoder không bị nhận sai dấu
            cmd_left = 0;
            cmd_right = 0;
            heading_initialized = false;
        }

        osDelay(20);
    }
    /* USER CODE END StartControlTask */
}

void StartCanTX(void const * argument)
{
    /* USER CODE BEGIN StartCanTX */
    uCAN_MSG tx_frame;
    memset(&tx_frame, 0, sizeof(tx_frame)); // <--- Thêm dòng dọn rác này
    tx_frame.frame.idType = dSTANDARD_CAN_MSG_ID_2_0B;

    for (;;)
    {
        // ==========================================
        // 1. GÓI 0x201: TỌA ĐỘ X VÀ Y (Thay thế cho vận tốc)
        // ==========================================
        tx_frame.frame.id = 0x201;
        tx_frame.frame.dlc = 8;
        memcpy(&tx_frame.frame.data0, &hOdom.pose.x, 4); // 4 byte cho X
        memcpy(&tx_frame.frame.data4, &hOdom.pose.y, 4); // 4 byte cho Y

        // BẮT BUỘC kẹp Mutex bảo vệ SPI
        if (osMutexWait(spiMutexHandle, 10) == osOK) {
            uint8_t tx_status = CANSPI_Transmit(&tx_frame);
            osMutexRelease(spiMutexHandle);

            can_tx_status = tx_status;
            if (tx_status) can_tx_count++;
        }
        osDelay(5); // Nghỉ 5ms để MCP2515 xả bộ đệm, tránh rớt gói tin

        // ==========================================
        // 2. GÓI 0x203: GÓC THETA
        // ==========================================
        tx_frame.frame.id = 0x203;
        tx_frame.frame.dlc = 4; // Chỉ cần 4 byte cho 1 biến float
        memcpy(&tx_frame.frame.data0, &hOdom.pose.theta, 4);

        if (osMutexWait(spiMutexHandle, 10) == osOK) {
            CANSPI_Transmit(&tx_frame);
            osMutexRelease(spiMutexHandle);
        }
        osDelay(5); // Tiếp tục nghỉ 5ms

        // ==========================================
        // 3. GÓI 0x202: SỰ KIỆN RẼ (Của Nhật, giữ nguyên)
        // ==========================================
        if (last_turn_direction != 0) {
            uCAN_MSG turn_frame;
            turn_frame.frame.idType = dSTANDARD_CAN_MSG_ID_2_0B;
            turn_frame.frame.id     = 0x202;
            turn_frame.frame.dlc    = 6;

            int16_t turn_dir = last_turn_direction;
            memcpy(&turn_frame.frame.data0, &turn_dir, 2);

            float yaw_val = mpu6050_yaw;
            memcpy(&turn_frame.frame.data2, &yaw_val, 4);

            if (osMutexWait(spiMutexHandle, 10) == osOK) {
                uint8_t turn_tx = CANSPI_Transmit(&turn_frame);
                osMutexRelease(spiMutexHandle);

                can_tx_status = turn_tx;
                if (turn_tx) can_tx_count++;
            }
            can_last_turn_yaw = yaw_val;
            last_turn_direction = 0; // Xóa cờ sau khi gửi
        }

        // ==========================================
        // CHU KỲ CẬP NHẬT
        // ==========================================
        osDelay(100); // FIX: Giảm từ 500ms xuống 100ms để vẽ map mượt mà 10Hz
    }
    /* USER CODE END StartCanTX */
}

void StartCanRX(void const * argument)
{
    /* USER CODE BEGIN StartCanRX */
    uCAN_MSG rx_frame;

    for (;;)
    {
        if (osSemaphoreWait(canRxSemaphoreHandle, osWaitForever) == osOK)
        {
            uint8_t rx_result = 0;

            // 1. KẸP MUTEX BẢO VỆ CỔNG SPI TRƯỚC KHI ĐỌC MCP2515
            if (osMutexWait(spiMutexHandle, 10) == osOK) {
                rx_result = CANSPI_Receive(&rx_frame);
                osMutexRelease(spiMutexHandle);
            }

            can_rx_status = rx_result;

            if (rx_result) {
                can_rx_count++;
                can_last_rx_id = rx_frame.frame.id;

                // 2. XỬ LÝ GÓI TIN ĐIỀU KHIỂN
                if (rx_frame.frame.id == 0x101)
                {
                    // Copy 4 byte đầu cho Vận tốc
                    memcpy(&target_velocity, &rx_frame.frame.data0, 4);
                    // SỬA LỖI: Copy 4 byte sau cho Góc (Bắt đầu từ data4)
                    memcpy(&target_angular,  &rx_frame.frame.data4, 4);
                }
            }
        }
    }
    /* USER CODE END StartCanRX */
}

void StartSensorTask(void const * argument)
{
    /* USER CODE BEGIN StartSensorTask */
    Servo_SetAngle(90);
    osDelay(500);

    for (;;)
    {
        float dist_front = HCSR04_GetDistance();

        if (dist_front > 0.0f && dist_front < 25.0f) {
            obstacle_detected = true; // Kích hoạt cờ báo dừng PID và khóa động cơ bánh
            Motor_SetSpeed(0, 0);
            osDelay(1000);

            Servo_SetAngle(180);
            osDelay(1000);
            float dist_left = HCSR04_GetDistance();

            Servo_SetAngle(0);
            osDelay(1000);
            float dist_right = HCSR04_GetDistance();

            Servo_SetAngle(90);
            osDelay(1000);

            if (dist_left > dist_right && dist_left > 30.0f) {
                Robot_TurnLeft();
            } else if (dist_right > dist_left && dist_right > 30.0f) {
                Robot_TurnRight();
            } else {
                Robot_TurnAround();
            }

            obstacle_detected = false; // Nhường quyền lại cho PID điều khiển đi thẳng
        }

        osDelay(50);
    }
    /* USER CODE END StartSensorTask */
}

/* ══════════════════════════════════════════════════════════════════════════
   Application functions — THÊM LỆNH RESET BỘ NHỚ LỖI NGAY SAU KHI RẼ XONG
   ══════════════════════════════════════════════════════════════════════════ */

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

void Servo_SetAngle(uint8_t angle)
{
    if (angle > 180) angle = 180;
    uint32_t pulse = 800 + ((uint32_t)angle * 70) / 9;
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, pulse);
}

void Motor_SetSpeed(int16_t speed_left, int16_t speed_right)
{

    if (speed_left >= 0) {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, speed_left);
    } else {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, -speed_left);
    }

    if (speed_right >= 0) {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, speed_right);
    } else {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, -speed_right);
    }
}

static float NormalizeYaw(float angle)
{
    while (angle > 180.0f)  angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;
    return angle;
}

// Lấy trung bình góc để tránh nhiễu do thân xe còn rung lắc
static float GetStableYaw(void)
{
    float yaw1 = mpu6050_yaw;
    osDelay(100);
    float yaw2 = mpu6050_yaw;
    return NormalizeYaw((yaw1 + yaw2) * 0.5f);
}

static void SendTurnFrame(int16_t turn_dir, float yaw_val)
{
    uCAN_MSG turn_frame;
    memset(&turn_frame, 0, sizeof(turn_frame)); // Dọn rác bộ nhớ trước khi nạp data

    turn_frame.frame.idType = dSTANDARD_CAN_MSG_ID_2_0B;
    turn_frame.frame.id     = 0x202;
    turn_frame.frame.dlc    = 6;

    memcpy(&turn_frame.frame.data0, &turn_dir, 2);
    memcpy(&turn_frame.frame.data2, &yaw_val, 4);

    // Kẹp Mutex bảo vệ SPI để tránh đụng độ với luồng CanTX
    if (osMutexWait(spiMutexHandle, 10) == osOK) {
        uint8_t tx = CANSPI_Transmit(&turn_frame);
        osMutexRelease(spiMutexHandle);
        if (tx) can_tx_count++;
    }
}
// ==========================================
// CÁC HÀM RẼ QUÁN TÍNH (MÙ) - TINH GỌN
// ==========================================

void Robot_TurnLeft(void)
{
    // 1. Cấp ga rẽ mù (Căn lại osDelay cho vừa 90 độ)
    Motor_SetSpeed(999,0);
    osDelay(200);
    Motor_SetSpeed(0, 0);

    // 2. Chờ thân xe bớt rung lắc cơ học
    osDelay(50);
    reset_imu_flag = true; // Ra lệnh cho ImuTask tự tính lại mốc 0 độ

    desired_yaw = 0.0f;
    YawPID_Reset(); // BẮT BUỘC PHẢI CÓ
    // 4. Bắn sự kiện lên CAN (Quy ước -90 là rẽ trái)
    SendTurnFrame(-90, desired_yaw);
    char uart_msg[80];
    int len = sprintf(uart_msg, "[TURN] dir=LEFT yaw=%.2f\r\n", desired_yaw);
    HAL_UART_Transmit(&huart1, (uint8_t*)uart_msg, len, 100);
}

void Robot_TurnRight(void)
{
    // 1. Cấp ga rẽ mù
    Motor_SetSpeed(0,999);
    osDelay(200);
    Motor_SetSpeed(0, 0);

    // 2. Chờ thân xe tĩnh lại
    osDelay(50);
    reset_imu_flag = true; // Ra lệnh cho ImuTask tự tính lại mốc 0 độ
    desired_yaw = 0.0f;
    YawPID_Reset(); // BẮT BUỘC PHẢI CÓ
    // 4. Bắn sự kiện lên CAN (Quy ước 90 là rẽ phải)
    SendTurnFrame(90, desired_yaw);
    char uart_msg[80];
    int len = sprintf(uart_msg, "[TURN] dir=RIGHT yaw=%.2f\r\n", desired_yaw);
    HAL_UART_Transmit(&huart1, (uint8_t*)uart_msg, len, 100);
}

void Robot_TurnAround(void)
{
    // 1. Cấp ga rẽ mù quay đầu
    Motor_SetSpeed(999, 0);
    osDelay(400);
    Motor_SetSpeed(0, 0);

    // 2. Chờ thân xe tĩnh lại
    osDelay(50);
    reset_imu_flag = true; // Ra lệnh cho ImuTask tự tính lại mốc 0 độ
    desired_yaw = 0.0f;
    YawPID_Reset(); // BẮT BUỘC PHẢI CÓ

    // 4. Bắn sự kiện lên CAN (Quy ước 180 là quay đầu)
    SendTurnFrame(180, desired_yaw);
    char uart_msg[80];
    int len = sprintf(uart_msg, "[TURN] dir=AROUND yaw=%.2f\r\n", desired_yaw);
    HAL_UART_Transmit(&huart1, (uint8_t*)uart_msg, len, 100);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == GPIO_PIN_0)
    {
        if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0) == GPIO_PIN_SET) {
            __HAL_TIM_SET_COUNTER(&htim5, 0);
            HAL_TIM_Base_Start(&htim5);
        } else {
            HAL_TIM_Base_Stop(&htim5);
            echo_time = __HAL_TIM_GET_COUNTER(&htim5);
            echo_done = true;
        }
    }
}

float HCSR04_GetDistance(void)
{
    echo_done = false;

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
    for (volatile int i = 0; i < 200; i++) { __NOP(); }
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);

    uint32_t start_tick = osKernelSysTick();
    while (echo_done == false)
    {
        if ((osKernelSysTick() - start_tick) > 30) {
            return 999.0f;
        }
        osDelay(1);
    }

    float distance = (echo_time * 0.0343f) / 2.0f;
    return distance;
}
/* USER CODE END Application */
