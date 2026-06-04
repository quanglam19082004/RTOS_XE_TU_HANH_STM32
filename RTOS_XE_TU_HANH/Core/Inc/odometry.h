/**
 * @file    odometry.h
 * @brief   Differential Drive Odometry - Data Structures & API
 * @author  Senior Embedded Software Engineer
 * @target  STM32F411CEU6 (Black Pill) + FreeRTOS
 *
 * Encoder Mode Timer Assignment (example, chỉnh lại cho đúng phần cứng):
 *   - Left  Encoder: TIM3 (CH1=PA6, CH2=PA7) — 16-bit
 *   - Right Encoder: TIM4 (CH1=PB6, CH2=PB7) — 16-bit
 *
 * Coordinate frame:
 *   X → trước (forward), Y → trái (left), θ = 0 là hướng trước (+X).
 */

#ifndef ODOMETRY_H
#define ODOMETRY_H

#ifdef __cplusplus
extern "C" {
#endif

/* =========================================================================
 * INCLUDES
 * ========================================================================= */
#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include <math.h>   /* sinf, cosf — ARM Cortex-M4 FPU tính nhanh */
#include <stdint.h>
#include <stdbool.h>

/* =========================================================================
 * COMPILE-TIME CONFIGURATION
 * ========================================================================= */

/** Tần số chạy Odometry Task (Hz) → chu kỳ dt = 1/ODOM_FREQ_HZ */
#define ODOM_FREQ_HZ            50U
#define ODOM_PERIOD_MS          (1000U / ODOM_FREQ_HZ)   /* 20 ms */
#define ODOM_DT_SEC             (1.0f / (float)ODOM_FREQ_HZ) /* 0.02 s */

/**
 * Timer counter width.
 * STM32F411: TIM2/TIM5 là 32-bit; TIM1/TIM3/TIM4/... là 16-bit.
 * Đặt thành 1 nếu dùng 16-bit timer (TIM3, TIM4).
 * Đặt thành 0 nếu dùng 32-bit timer (TIM2, TIM5).
 */
#define ENCODER_TIMER_16BIT     1   /* 1 = 16-bit, 0 = 32-bit */

/* =========================================================================
 * KIỂU DỮ LIỆU PHỤ TRỢ
 * ========================================================================= */

/**
 * @brief  Kiểu đếm xung phù hợp với độ rộng timer.
 *         - 16-bit timer: uint16_t  (wrap tự nhiên ở 65535)
 *         - 32-bit timer: uint32_t  (wrap tự nhiên ở 4294967295)
 */
#if ENCODER_TIMER_16BIT
    typedef uint16_t encoder_cnt_t;
    typedef int16_t  encoder_delta_t;   /* Có dấu, xử lý underflow/overflow */
#else
    typedef uint32_t encoder_cnt_t;
    typedef int32_t  encoder_delta_t;
#endif

/* =========================================================================
 * STRUCT 1: Thông số cơ khí Robot
 * ========================================================================= */

/**
 * @brief  Tham số vật lý / cơ khí của robot.
 *
 * Điền vào đúng giá trị đo được từ phần cứng thực tế.
 * Tất cả đơn vị SI (mét, radian).
 */
typedef struct {
    float wheel_diameter_m;     /**< D: Đường kính bánh xe [m]               */
    float wheel_base_m;         /**< L: Khoảng cách tâm 2 bánh [m]           */
    float wheel_circumference_m;/**< π*D, tính sẵn để tránh nhân lặp          */
    uint32_t ticks_per_rev;     /**< N: Số xung encoder / 1 vòng bánh        */
                                /**<   = (PPR * gear_ratio * mode_multiplier) */
                                /**<   PPR: xung/vòng motor, gear: tỉ số hộp */
                                /**<   mode_mult: 1(1x), 2(2x), 4(4x mode)   */
    float meters_per_tick;      /**< = wheel_circumference_m / ticks_per_rev */
                                /**<   tính sẵn để dùng trong vòng lặp       */
} RobotParams_t;

/* =========================================================================
 * STRUCT 2: Cấu hình phần cứng Encoder (Timer HAL)
 * ========================================================================= */

/**
 * @brief  Ánh xạ giữa encoder vật lý và Timer HAL handle.
 */
typedef struct {
    TIM_HandleTypeDef *htim_left;   /**< Timer handle kênh encoder TRÁI  */
    TIM_HandleTypeDef *htim_right;  /**< Timer handle kênh encoder PHẢI  */
} EncoderHW_t;

/* =========================================================================
 * STRUCT 3: Trạng thái runtime của Encoder (tick counts)
 * ========================================================================= */

/**
 * @brief  Lưu giá trị đếm trước đó của 2 encoder để tính delta.
 *
 * Kỹ thuật xử lý overflow/underflow 16-bit:
 *   delta = (int16_t)(current - previous)
 *   → Phép ép kiểu có dấu xử lý wrap-around tự động nhờ modular arithmetic.
 *   Ví dụ: current=5, previous=65530 → (int16_t)(5 - 65530) = +11 (đúng)
 */
typedef struct {
    encoder_cnt_t   prev_left;      /**< Giá trị CNT trước của Timer TRÁI  */
    encoder_cnt_t   prev_right;     /**< Giá trị CNT trước của Timer PHẢI  */
    encoder_delta_t delta_left;     /**< Số xung thay đổi trong 1 chu kỳ   */
    encoder_delta_t delta_right;    /**< Số xung thay đổi trong 1 chu kỳ   */
    int32_t         total_left;     /**< Tổng xung cộng dồn (không wrap)    */
    int32_t         total_right;    /**< Tổng xung cộng dồn (không wrap)    */
} EncoderState_t;

/* =========================================================================
 * STRUCT 4: Tọa độ tư thế Robot (Pose)
 * ========================================================================= */

/**
 * @brief  Tư thế robot trong hệ tọa độ toàn cục (Global Frame).
 *
 * Quy ước:
 *   - (x, y): vị trí tâm robot [m]
 *   - theta : góc heading [radian], 0 = hướng +X, tăng theo chiều CCW
 */
typedef struct {
    float x;       /**< Tọa độ X toàn cục [m]       */
    float y;       /**< Tọa độ Y toàn cục [m]       */
    float theta;   /**< Góc heading toàn cục [rad]  */
} RobotPose_t;

/* =========================================================================
 * STRUCT 5: Vận tốc tức thời Robot
 * ========================================================================= */

/**
 * @brief  Vận tốc tuyến tính và góc tính được từ encoder sau mỗi chu kỳ.
 */
typedef struct {
    float linear_mps;   /**< Vận tốc thẳng [m/s]   */
    float angular_rps;  /**< Vận tốc góc   [rad/s] */
} RobotVelocity_t;

/* =========================================================================
 * STRUCT 6: Handle tổng hợp của module Odometry (Object-like pattern)
 * ========================================================================= */

/**
 * @brief  Đối tượng trung tâm quản lý toàn bộ trạng thái Odometry.
 *
 * Khởi tạo bằng Odometry_Init() trước khi gọi bất cứ hàm nào khác.
 * Truyền con trỏ vào StartOdometryTask và các module khác (CAN, Control).
 */
typedef struct {
    RobotParams_t   params;     /**< Thông số cơ khí (read-only sau init)   */
    EncoderHW_t     hw;         /**< Timer HAL handles                      */
    EncoderState_t  enc;        /**< Trạng thái encoder runtime             */
    RobotPose_t     pose;       /**< Tọa độ tư thế hiện tại (output chính) */
    RobotVelocity_t velocity;   /**< Vận tốc tức thời (output phụ)         */
    bool            initialized;/**< Guard: bảo vệ khỏi dùng trước init    */
} OdometryHandle_t;

/* =========================================================================
 * API — FUNCTION DECLARATIONS
 * ========================================================================= */

/**
 * @brief  Khởi tạo OdometryHandle với thông số cơ khí và hardware.
 * @param  hOdom   Con trỏ đến OdometryHandle_t do người dùng cấp phát.
 * @param  diameter_m   Đường kính bánh xe [m]
 * @param  wheelbase_m  Khoảng cách 2 bánh  [m]
 * @param  ticks_per_rev Số xung/vòng (sau gear ratio + encoder mode)
 * @param  htim_L   HAL Timer handle — encoder trái
 * @param  htim_R   HAL Timer handle — encoder phải
 * @retval HAL_OK / HAL_ERROR
 */
HAL_StatusTypeDef Odometry_Init(OdometryHandle_t *hOdom,
                                float diameter_m,
                                float wheelbase_m,
                                uint32_t ticks_per_rev,
                                TIM_HandleTypeDef *htim_L,
                                TIM_HandleTypeDef *htim_R);

/**
 * @brief  Reset tọa độ pose về (0, 0, 0) hoặc giá trị tùy chỉnh.
 */
void Odometry_ResetPose(OdometryHandle_t *hOdom,
                        float x, float y, float theta_rad);

/**
 * @brief  Đọc raw encoder counts, tính delta, xử lý overflow.
 *         Gọi nội bộ bởi StartOdometryTask.
 */
void Odometry_ReadEncoders(OdometryHandle_t *hOdom);

/**
 * @brief  Tính toán kinematics và cập nhật pose.
 *         Gọi nội bộ bởi StartOdometryTask.
 */
void Odometry_UpdatePose(OdometryHandle_t *hOdom, float dt);

/**
 * @brief  FreeRTOS Task function — entry point cho scheduler.
 * @param  argument  Con trỏ truyền từ osThreadCreate(), ép kiểu về OdometryHandle_t*.
 */
void StartOdometryTask(void const * argument);
/**
 * @brief  Thread-safe getter: lấy pose hiện tại (copy by value).
 *         Dùng taskENTER_CRITICAL nên chỉ gọi từ task khác, KHÔNG gọi từ ISR.
 * @param  hOdom   Nguồn dữ liệu
 * @param  out     Buffer nhận kết quả
 */
void Odometry_GetPose(const OdometryHandle_t *hOdom, RobotPose_t *out);

/**
 * @brief  Thread-safe getter: lấy velocity hiện tại.
 */
void Odometry_GetVelocity(const OdometryHandle_t *hOdom, RobotVelocity_t *out);

#ifdef __cplusplus
}
#endif

#endif /* ODOMETRY_H */
