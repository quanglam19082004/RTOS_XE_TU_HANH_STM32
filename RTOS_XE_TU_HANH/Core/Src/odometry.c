/**
 * @file    odometry.c
 * @brief   Differential Drive Odometry — Full Implementation
 * @author  Senior Embedded Software Engineer
 * @target  STM32F411CEU6 (Black Pill) + FreeRTOS
 *
 * ┌──────────────────────────────────────────────────────────────────────┐
 * │                     LUỒNG XỬ LÝ ODOMETRY                           │
 * │                                                                      │
 * │  [Timer CNT Reg]──►Odometry_ReadEncoders()──►delta_L, delta_R       │
 * │                              │                                       │
 * │                              ▼                                       │
 * │                   Odometry_UpdatePose()                             │
 * │                    │                                                 │
 * │          S_L = delta_L * meters_per_tick                            │
 * │          S_R = delta_R * meters_per_tick                            │
 * │          ΔS  = (S_R + S_L) / 2         ← quãng đường trung bình    │
 * │          Δθ  = (S_R - S_L) / L         ← góc xoay                  │
 * │          X  += ΔS * cos(θ + Δθ/2)     ← mid-point integration      │
 * │          Y  += ΔS * sin(θ + Δθ/2)                                   │
 * │          θ  += Δθ                                                    │
 * └──────────────────────────────────────────────────────────────────────┘
 *
 * Ghi chú tối ưu hóa:
 *   - STM32F411 có FPU (Cortex-M4F) → float nhanh như int trên desktop.
 *     Enable FPU trong CubeMX: Cortex-M4 FPU → Enabled (FPU_PRESENT=1,
 *     __FPU_USED=1). Compiler flag: -mfpu=fpv4-sp-d16 -mfloat-abi=hard
 *   - Dùng sinf/cosf (32-bit) thay sinl/cosl (64-bit) — tiết kiệm ~40%.
 *   - meters_per_tick tính sẵn tại init → vòng lặp chỉ nhân 1 lần.
 *   - Critical section ngắn: chỉ bao quanh đoạn copy pose ra ngoài.
 *   - vTaskDelayUntil đảm bảo drift-free 50Hz dù xử lý mất vài µs.
 */

#include "odometry.h"
#include <string.h>  /* memset */

/* =========================================================================
 * PRIVATE HELPERS — Forward declarations (inline, linkage = translation unit)
 * ========================================================================= */
void Odometry_ReadEncoders(OdometryHandle_t *hOdom);
void Odometry_UpdatePose(OdometryHandle_t *hOdom, float dt);
static inline float Odometry_WrapToPi(float angle);

/* =========================================================================
 * PUBLIC: Odometry_Init
 * ========================================================================= */

/**
 * @brief  Khởi tạo module Odometry.
 *
 * Sau hàm này, gọi HAL_TIM_Encoder_Start() trước khi chạy task.
 * Ví dụ trong main.c sau MX_TIMx_Init():
 *   HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
 *   HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
 *   Odometry_Init(&hOdom, 0.065f, 0.155f, 1560, &htim3, &htim4);
 */
HAL_StatusTypeDef Odometry_Init(OdometryHandle_t *hOdom,
                                float diameter_m,
                                float wheelbase_m,
                                uint32_t ticks_per_rev,
                                TIM_HandleTypeDef *htim_L,
                                TIM_HandleTypeDef *htim_R)
{
    /* Kiểm tra con trỏ null */
    if (hOdom == NULL || htim_L == NULL || htim_R == NULL) {
        return HAL_ERROR;
    }
    if (diameter_m <= 0.0f || wheelbase_m <= 0.0f || ticks_per_rev == 0U) {
        return HAL_ERROR;
    }

    /* Zero-init toàn bộ struct */
    memset(hOdom, 0, sizeof(OdometryHandle_t));

    /* Thông số cơ khí */
    hOdom->params.wheel_diameter_m      = diameter_m;
    hOdom->params.wheel_base_m          = wheelbase_m;
    hOdom->params.ticks_per_rev         = ticks_per_rev;
    hOdom->params.wheel_circumference_m = (float)M_PI * diameter_m;

    /* *** Key optimization: tính meters_per_tick một lần duy nhất *** */
    hOdom->params.meters_per_tick =
        hOdom->params.wheel_circumference_m / (float)ticks_per_rev;

    /* Hardware handles */
    hOdom->hw.htim_left  = htim_L;
    hOdom->hw.htim_right = htim_R;

    /* Snapshot giá trị CNT ban đầu để delta đầu tiên = 0 */
    hOdom->enc.prev_left  = (encoder_cnt_t)__HAL_TIM_GET_COUNTER(htim_L);
    hOdom->enc.prev_right = (encoder_cnt_t)__HAL_TIM_GET_COUNTER(htim_R);

    hOdom->initialized = true;

    return HAL_OK;
}

/* =========================================================================
 * PUBLIC: Odometry_ResetPose
 * ========================================================================= */
void Odometry_ResetPose(OdometryHandle_t *hOdom,
                        float x, float y, float theta_rad)
{
    if (hOdom == NULL) return;

    /* Dùng critical section vì pose có thể đang được đọc bởi task khác */
    taskENTER_CRITICAL();
    {
        hOdom->pose.x     = x;
        hOdom->pose.y     = y;
        hOdom->pose.theta = theta_rad;
    }
    taskEXIT_CRITICAL();
}

/* =========================================================================
 * PRIVATE: Odometry_ReadEncoders
 *
 * Đọc thanh ghi CNT của 2 Timer và tính delta_ticks.
 *
 * KỸ THUẬT XỬ LÝ OVERFLOW / UNDERFLOW (16-bit timer):
 * ──────────────────────────────────────────────────────
 * Timer 16-bit đếm từ 0 → 65535 (Encoder Mode, cả 2 hướng).
 * Khi bánh quay thuận: 65533 → 65534 → 65535 → 0 → 1 → 2  (overflow)
 * Khi bánh quay nghịch: 2 → 1 → 0 → 65535 → 65534 → 65533 (underflow)
 *
 * Phép ép kiểu (int16_t)(current - prev) tự xử lý nhờ C modular arithmetic:
 *   Overflow:  current=2,     prev=65534 → (int16_t)(2 - 65534) = +4   ✓
 *   Underflow: current=65534, prev=2     → (int16_t)(65534 - 2) = -4   ✓
 *
 * Điều kiện: |delta| < 32768 mỗi chu kỳ 20ms (đủ với robot tốc độ thông thường).
 * Nếu robot cực nhanh, chuyển sang 32-bit timer (TIM2/TIM5) và dùng int32_t.
 * ========================================================================= */
void Odometry_ReadEncoders(OdometryHandle_t *hOdom)
{
    /* Đọc thanh ghi CNT (atomic trên Cortex-M) */
//    encoder_cnt_t cur_left  = (encoder_cnt_t)__HAL_TIM_GET_COUNTER(hOdom->hw.htim_left);
//    encoder_cnt_t cur_right = (encoder_cnt_t)__HAL_TIM_GET_COUNTER(hOdom->hw.htim_right);

    /*
     * Ép kiểu có dấu để xử lý overflow/underflow tự động.
     * Với 16-bit: (int16_t)(uint16_t - uint16_t) → wrap-around arithmetic.
     * Với 32-bit: (int32_t)(uint32_t - uint32_t) → tương tự.
     */
//    hOdom->enc.delta_left  = (encoder_delta_t)(cur_left  - hOdom->enc.prev_left);
//    hOdom->enc.delta_right = (encoder_delta_t)(cur_right - hOdom->enc.prev_right);
//
//    /* Lưu lại cho chu kỳ kế tiếp */
//    hOdom->enc.prev_left  = cur_left;
//    hOdom->enc.prev_right = cur_right;
//
//    /* Cộng dồn tổng (int32_t không overflow trong phạm vi hoạt động thực tế) */
//    hOdom->enc.total_left  += (int32_t)hOdom->enc.delta_left;
//    hOdom->enc.total_right += (int32_t)hOdom->enc.delta_right;
}

/* =========================================================================
 * PRIVATE: Odometry_UpdatePose
 *
 * Áp dụng phương trình động học Differential Drive.
 *
 * PHƯƠNG TRÌNH:
 * ─────────────────────────────────────────────────────────────────────────
 *   S_L = delta_L * meters_per_tick       [m] quãng đường bánh trái
 *   S_R = delta_R * meters_per_tick       [m] quãng đường bánh phải
 *
 *   ΔS  = (S_R + S_L) / 2                [m] quãng đường trung bình
 *   Δθ  = (S_R - S_L) / L                [rad] góc quay
 *
 *   θ_mid = θ + Δθ/2                     [rad] góc trung gian (mid-point)
 *
 *   X  += ΔS * cos(θ_mid)                [m]
 *   Y  += ΔS * sin(θ_mid)                [m]
 *   θ  += Δθ  (rồi normalize về (-π, π]) [rad]
 *
 * Lý do dùng θ_mid (midpoint integration) thay vì θ:
 *   Chính xác hơn khi robot vừa đi vừa quay, sai số O(dt²) thay vì O(dt).
 * ========================================================================= */
void Odometry_UpdatePose(OdometryHandle_t *hOdom, float dt)
{
    /* ── Bước 1: Quy đổi xung → quãng đường [m] ─────────────────────── */
    const float mpt = hOdom->params.meters_per_tick; /* đã tính sẵn */

    float s_left  = (float)hOdom->enc.delta_left  * mpt;
    float s_right = (float)hOdom->enc.delta_right * mpt;

    /* ── Bước 2: Tính quãng đường trung bình và góc xoay ─────────────── */
    float delta_s     = (s_right + s_left)  * 0.5f;                       /* ΔS  */
    float delta_theta = (s_right - s_left)  / hOdom->params.wheel_base_m; /* Δθ  */

    /* ── Bước 3: Cập nhật vận tốc tức thời ──────────────────────────── */
    /* dt = ODOM_DT_SEC (constant), tránh division mỗi vòng → truyền vào */
    hOdom->velocity.linear_mps  = delta_s     / dt;
    hOdom->velocity.angular_rps = delta_theta / dt;

    /* ── Bước 4: Midpoint integration → cập nhật pose ───────────────── */
    /*
     * θ_mid = θ_current + Δθ/2
     * Dùng θ_mid để tính hướng di chuyển thay vì θ_current,
     * giảm sai số tích phân khi robot vừa tịnh tiến vừa xoay.
     */
    float theta_mid = hOdom->pose.theta + (delta_theta * 0.5f);

    /*
     * sinf/cosf: dùng bản 32-bit float, ARM FPU tính trong ~14 cycles.
     * KHÔNG dùng sin/cos (64-bit double) trên MCU — chậm 10x.
     */
    hOdom->pose.x += delta_s * cosf(theta_mid);
    hOdom->pose.y += delta_s * sinf(theta_mid);
    hOdom->pose.theta = Odometry_WrapToPi(hOdom->pose.theta + delta_theta);
}

/* =========================================================================
 * PRIVATE: Odometry_WrapToPi
 * Chuẩn hóa góc về khoảng (-π, π] để tránh theta tăng vô hạn.
 * ========================================================================= */
static inline float Odometry_WrapToPi(float angle)
{
    /* Dùng vòng lặp thay fmodf để tránh overhead trên MCU */
    while (angle >  (float)M_PI) angle -= 2.0f * (float)M_PI;
    while (angle <= -(float)M_PI) angle += 2.0f * (float)M_PI;
    return angle;
}

/* =========================================================================
 * PUBLIC: Thread-safe getters (dùng ở task CAN, Control, Debug)
 * ========================================================================= */
void Odometry_GetPose(const OdometryHandle_t *hOdom, RobotPose_t *out)
{
    if (hOdom == NULL || out == NULL) return;
    taskENTER_CRITICAL();
    {
        *out = hOdom->pose; /* struct copy — atomic trong critical section */
    }
    taskEXIT_CRITICAL();
}

void Odometry_GetVelocity(const OdometryHandle_t *hOdom, RobotVelocity_t *out)
{
    if (hOdom == NULL || out == NULL) return;
    taskENTER_CRITICAL();
    {
        *out = hOdom->velocity;
    }
    taskEXIT_CRITICAL();
}

/* =========================================================================
 * PUBLIC: StartOdometryTask — FreeRTOS Task Entry Point
 *
 * Cách khai báo và tạo task trong main.c (hoặc freertos.c của CubeMX):
 *
 *   // Khai báo global handle (hoặc dùng extern)
 *   OdometryHandle_t hOdom;
 *
 *   // Trong main() sau MX_TIMx_Init():
 *   HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);  // Timer trái
 *   HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);  // Timer phải
 *   Odometry_Init(&hOdom,
 *                 0.065f,   // D = 65mm bánh xe
 *                 0.155f,   // L = 155mm khoảng cách 2 bánh
 *                 1560,     // N = 390 PPR * 4 (quadrature mode)
 *                 &htim3, &htim4);
 *
 *   // Tạo FreeRTOS Task:
 *   xTaskCreate(StartOdometryTask,
 *               "OdometryTask",
 *               256,          // Stack words (điều chỉnh nếu cần)
 *               (void*)&hOdom,
 *               3,            // Priority: cao hơn Idle, thấp hơn Control
 *               NULL);
 *
 *   vTaskStartScheduler();
 * ========================================================================= */
void StartOdometryTask(void const *argument)
{
    /* ── Lấy handle từ argument ────────────────────────────────────────── */
    OdometryHandle_t *hOdom = (OdometryHandle_t *)argument;

    /* Guard: bảo vệ nếu quên gọi Odometry_Init() */
    configASSERT(hOdom != NULL);
    configASSERT(hOdom->initialized == true);

    /* ── Hằng số thời gian (const → compiler lưu vào flash, không stack) ─ */
    const float dt              = ODOM_DT_SEC;                /* 0.02 s */
    const TickType_t period     = pdMS_TO_TICKS(ODOM_PERIOD_MS); /* 20ms */

    /* ── vTaskDelayUntil: khởi tạo mốc thời gian trước khi vào vòng lặp ─ */
    /*
     * Cách dùng đúng của vTaskDelayUntil:
     *   xLastWakeTime lưu thời điểm task ĐƯỢC PHÉP chạy.
     *   Sau mỗi vòng, xLastWakeTime += period → drift-free 50Hz.
     *   Khác vTaskDelay(period): vTaskDelay tính từ lúc THỰC SỰ chạy
     *   → bị drift nếu xử lý mất nhiều thời gian.
     */
    TickType_t xLastWakeTime = xTaskGetTickCount();

    /* ═══════════════════════════════════════════════════════════════════
     * VÒNG LẶP CHÍNH — 50 Hz
     * ═══════════════════════════════════════════════════════════════════ */
    for (;;)
    {
        /* ── [1] Block đến đúng chu kỳ tiếp theo ─────────────────────── */
        /*
         * Task bị block tại đây cho đến khi xLastWakeTime + period.
         * Nếu phần xử lý bên dưới mất < 20ms → task ngủ phần còn lại.
         * Nếu mất > 20ms (overrun) → vTaskDelayUntil chạy ngay lập tức,
         *   FreeRTOS tự điều chỉnh để catch up.
         */
        vTaskDelayUntil(&xLastWakeTime, period);

        /* ── [2] Đọc Encoder — tính delta_ticks ────────────────────── */
        /*
         * Hàm này chỉ đọc thanh ghi và làm phép trừ.
         * Overhead: ~10 cycles (đọc 2 register + 2 phép trừ).
         * KHÔNG cần disable interrupt vì:
         *   - __HAL_TIM_GET_COUNTER là đọc 1 register (atomic trên M4)
         *   - Encoder Mode timer update xảy ra độc lập với CPU
         */
        Odometry_ReadEncoders(hOdom);

        /* ── [3] Tính Kinematics — cập nhật Pose ───────────────────── */
        /*
         * Hàm này thực hiện:
         *   2 phép nhân float (quy xung → mét)
         *   2 phép cộng/trừ, 1 phép chia
         *   1 sinf + 1 cosf (FPU: ~14 cycles mỗi hàm)
         *   2 phép cộng dồn pose
         *   1 vòng lặp normalize góc (thường 0-1 iteration)
         * Tổng: ~100-200 cycles @ 100MHz ≈ 1-2 µs
         */
        Odometry_UpdatePose(hOdom, dt);

        /* ── [4] Debug / Monitoring hook (tùy chọn) ─────────────────── */
        /*
         * Bỏ comment để dùng, hoặc xóa khi deploy.
         * Ví dụ: gửi qua UART/ITM để vẽ đồ thị trên Serial Plotter.
         *
         * char buf[80];
         * snprintf(buf, sizeof(buf), "X:%.3f Y:%.3f T:%.3f\r\n",
         *          hOdom->pose.x, hOdom->pose.y, hOdom->pose.theta);
         * HAL_UART_Transmit(&huart1, (uint8_t*)buf, strlen(buf), 10);
         */

    } /* end for(;;) */

    /* Không bao giờ đến đây — FreeRTOS task không được return */
    /* Nếu cần thoát: vTaskDelete(NULL); */
}

/* =========================================================================
 * END OF FILE
 * =========================================================================
 *
 * HƯỚNG DẪN TÍCH HỢP VỚI CÁC MODULE KHÁC:
 * ─────────────────────────────────────────
 * Module 3 (CAN / MCP2515):
 *   RobotPose_t pose;
 *   Odometry_GetPose(&hOdom, &pose);
 *   // đóng gói pose vào CAN frame và gửi đi
 *
 * Module 2 (Obstacle Avoidance):
 *   RobotPose_t pose;
 *   Odometry_GetPose(&hOdom, &pose);
 *   // dùng pose.x, pose.y, pose.theta để quyết định hướng né
 *
 * THÔNG SỐ N (ticks_per_rev) — CÁCH TÍNH:
 * ─────────────────────────────────────────
 *   N = PPR × gear_ratio × encoder_mode
 *   Ví dụ: Encoder 11 PPR, hộp số 1:30, Quadrature 4x mode
 *   N = 11 × 30 × 4 = 1320 ticks/vòng bánh
 *
 *   encoder_mode:
 *     TIM_ENCODERMODE_TI1     → 1x (đếm theo cạnh CH1)      → mode = 1
 *     TIM_ENCODERMODE_TI2     → 1x (đếm theo cạnh CH2)      → mode = 1
 *     TIM_ENCODERMODE_TI12    → 4x (đếm cả 4 cạnh A+B)      → mode = 4
 *     (2x mode không có sẵn trong HAL, dùng TI12 và chia 2 trong code)
 *
 * KIỂM TRA CHIỀU ENCODER:
 * ─────────────────────────
 *   Đẩy robot về phía trước một đoạn, in ra delta_left và delta_right.
 *   Cả 2 phải dương. Nếu âm → đổi chiều: hoán đổi CH1/CH2 của encoder đó
 *   HOẶC đổi dấu trong code: delta_left = -delta_left;
 * ========================================================================= */
