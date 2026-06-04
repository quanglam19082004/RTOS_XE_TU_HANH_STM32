# Phân Tích Chi Tiết: Giải Pháp PID Cho Yaw Control

## 1. VẤN ĐỀ PHÁT HIỆN

### Triệu Chứng
- Robot đi lệch khi chạy thẳng
- Không tự quay trở lại đường thẳng ban đầu
- Chỉ "giảm bớt" độ lệch, không "khắc phục"

### Root Cause: P-Only Controller
```c
// CÓ:
float yaw_error = desired_yaw - mpu6050_yaw;
float correction = Kp * yaw_error;  // <-- KHI ERROR → 0, CORRECTION → 0
cmd_left = 600 + correction;
cmd_right = 600 - correction;
```

**Vấn đề:**
- Khi `yaw_error ≈ 0`, bộ điều khiển dừng hiệu chỉnh
- Robot không bao giờ quay trở lại `desired_yaw` mà chỉ dừng khi sai số hiện tại giảm
- Nó "học" để dịch chuyển mục tiêu mới thay vì quay trở lại cái cũ

### Mô Phỏng Hành Động

```
desired_yaw = 0°
current_yaw = 0° → error = 0° → correction = 0 ✓ OK

[Motor không hoàn toàn đối xứng, drift bắt đầu]

current_yaw = 5° → error = -5° → correction = -Kp*5 = -10
  → Motor_Left bị giảm, Motor_Right tăng
  → Robot quay trở lại

current_yaw = 3° → error = -3° → correction = -Kp*3 = -6
  → Hiệu chỉnh yếu hơn

current_yaw = 1° → error = -1° → correction = -Kp*1 = -2
  → Hiệu chỉnh quá yếu, robot vẫn lệch

current_yaw = 0.5° → error = -0.5° → correction = -Kp*0.5 = -1
  → Mấy mà không đủ mạnh? STOP CORRECTING
  → Robot tiếp tục lệch nhẹ, không bao giờ quay trở lại!

current_yaw = 0.6° → error = -0.6° → ...
  → VÒNG LẶP VĨNH VIỄN ở chế độ lệch nhẹ
```

## 2. GIẢI PHÁP: PID CONTROLLER

### Công Thức PID Tổng Quát

$$
u(t) = K_p \cdot e(t) + K_i \int_0^t e(\tau) d\tau + K_d \frac{de(t)}{dt}
$$

Hay dưới dạng rời rạc (discrete):

$$
\text{correction} = K_p \cdot e_n + K_i \cdot \sum_{i=0}^{n} e_i + K_d \cdot \frac{e_n - e_{n-1}}{\Delta t}
$$

### Ba Thành Phần

#### 1. **P Term (Proportional)**
```c
p_term = Kp * yaw_error;
```
- **Mục đích:** Phản ứng nhanh với sai số hiện tại
- **Ưu:** Nhanh chóng
- **Nhược:** Không bao giờ khắc phục được sai số hoàn toàn (=0)

#### 2. **I Term (Integral)** ← **ĐÂY LÀ CHÌA KHÓA**
```c
error_integral += yaw_error * dt;  // Tích lũy sai số theo thời gian
i_term = Ki * error_integral;
```
- **Mục đích:** BUỘC robot quay trở lại, ngay cả khi sai số hiện tại nhỏ
- **Cơ chế:** Tích lũy sai số theo thời gian
  - Nếu robot lệch 5°, tích lũy = 5°
  - Nếu nó vẫn lệch sau 1 giây, tích lũy = 5 + 5 = 10°
  - Hiệu chỉnh = Ki * 10° > Ki * 5° (mạnh hơn!)
- **Kết quả:** Robot không thể "thoát khỏi" sai số nhỏ

#### 3. **D Term (Derivative)**
```c
d_term = Kd * (yaw_error - yaw_error_prev) / dt;  // Tốc độ thay đổi sai số
```
- **Mục đích:** Làm mềm mại, giảm dao động
- **Ưu:** Ngăn ngừa quá hiệu chỉnh (overshoot)
- **Nhược:** Dễ nhiễu từ cảm biến

### So Sánh

```
Thời gian (s)    Yaw (°)   Error   P_term   I_term    Correction
                          (°)     (Kp=2)   (Ki=0.3)
─────────────────────────────────────────────────────────────────
0                 0         0       0        0          0
1                 5        -5      -10      -1.5      -11.5  ← Tích lũy bắt đầu
2                 4        -4       -8      -3.0      -11.0
3                 2        -2       -4      -3.9      -7.9
4                 0.5      -0.5     -1      -4.2      -5.2  ← I term vẫn buộc
5                 0        0         0      -4.2      -4.2  ← I term cuối cùng quay được
                                                              robot trở lại
6                -0.3     +0.3     +0.6    -4.1      -3.5  ← P term ngăn quay quá
...
Stable            0         0       0        0          0  ← Cân bằng
```

**Với P-only:**
- Dừng hiệu chỉnh khi error = 0.5° (không đủ mạnh)
- Robot không bao giờ quay trở lại

**Với PI:**
- I term tích lũy mặc dù error nhỏ
- Buộc robot phải quay trở lại đến khi error = 0 hoàn toàn

## 3. IMPLEMENTATION CHI TIẾT

### 3.1 Anti-Windup

**Vấn đề:** Nếu sai số không về 0 (ví dụ do motor bị kẹt), `error_integral` sẽ tích lũy vô hạn
```c
error_integral += yaw_error * dt;

if (error_integral > 150.0f) {
    error_integral = 150.0f;  // <-- Giới hạn!
}
```

**Hiệu ứng:** Hiệu chỉnh có giới hạn trên, không thể quay quá mạnh

### 3.2 Output Clamping
```c
float correction = p_term + i_term + d_term;

if (correction > 400.0f) {
    correction = 400.0f;  // <-- Không được quay quá mạnh
}

cmd_left = 600 + correction;   // [0, 999]
cmd_right = 600 - correction;  // [0, 999]
```

### 3.3 Reset PID Khi Desired Yaw Thay Đổi
```c
if (new_desired_yaw != yaw_pid.desired_yaw) {
    YawPID_Reset();  // error_integral = 0, error_prev = 0
    yaw_pid.desired_yaw = new_desired_yaw;
}
```

**Lý do:** Tránh "sốc" từ integral cũ khi nhảy đến waypoint mới

## 4. TUNING PARAMETERS

### Giá Trị Mặc Định
```c
.Kp = 2.0f;          // Phản ứng nhanh
.Ki = 0.3f;          // Tích lũy từ từ (có thể tăng nếu cần)
.Kd = 0.5f;          // Làm mềm
.integral_max = 150.0f;   // Anti-windup
.output_max = 400.0f;     // Giới hạn hiệu chỉnh
```

### Quy Luật Tăng/Giảm

| Hành Vi | Kp ↑ | Ki ↑ | Kd ↑ |
|--------|------|------|------|
| Phản ứng nhanh hơn | ✓ | ✓ | ✗ |
| Giảm sai số tĩnh | ✗ | ✓ | ✗ |
| Ít dao động | ✗ | ✗ | ✓ |
| Ổn định hơn | ✗ | ✗ | ✓ |

## 5. VÌ SAO ĐIỀU NÀY HOẠT ĐỘNG

### Trước Đây (P-only):
```
Robot lệch 5° → correction = -10
Robot lệch 2° → correction = -4
Robot lệch 1° → correction = -2  ← DỪNG, không đủ mạnh
Robot tiếp tục lệch, không có cách nào để quay trở lại
```

### Bây Giờ (PI):
```
Robot lệch 5° tại t=0 → integral = 5° → correction = -10 - 1.5 = -11.5 ← MẠNH
Robot lệch 2° tại t=1s → integral = 7° → correction = -4 - 2.1 = -6.1  ← VẪN MẠNH
Robot lệch 0.5° tại t=2s → integral = 7.5° → correction = -1 - 2.25 = -3.25 ← KHÔNG DỪNG!
Robot bị buộc quay trở lại cho đến khi error = 0 hoàn toàn
```

**Integral term là "bộ nhớ" của hệ thống - nó không bao giờ "quên" được sai số!**

## 6. EXPECTED BEHAVIOR

### Sau Khi Fix:

1. **Khi robot đi thẳng:**
   - Phát hiện lệch → P term phản ứng nhanh
   - Nếu vẫn lệch → I term tích lũy, buộc quay trở lại
   - Kết quả: Robot tự động quay trở lại đường thẳng ban đầu

2. **Debug Output Minh Họa:**
   ```
   [PID] Desired:0.00 Current:3.50 Error:-3.50 | P:-7.0 I:-1.2 D:-0.3 Correction:-8.5
   [PID] Desired:0.00 Current:2.10 Error:-2.10 | P:-4.2 I:-2.4 D:-0.4 Correction:-7.0
   [PID] Desired:0.00 Current:0.80 Error:-0.80 | P:-1.6 I:-3.1 D:-0.3 Correction:-5.0
   [PID] Desired:0.00 Current:0.00 Error:0.00  | P:0.0  I:-2.8 D:0.0  Correction:-0.8
   [PID] Desired:0.00 Current:0.05 Error:-0.05 | P:-0.1 I:-2.9 D:0.0  Correction:-3.0
   ```
   - Robot từ từ quay trở lại, error giảm từ -3.5° → 0°

## 7. DEBUG & TROUBLESHOOTING

### Nếu vẫn lệch:
```c
// Tăng Ki để tích lũy mạnh hơn
.Ki = 0.5f;  // Từ 0.3f
```

### Nếu dao động (zigzag):
```c
// Tăng Kd để làm mềm
.Kd = 0.8f;  // Từ 0.5f
```

### Nếu quay quá mạnh:
```c
// Giảm Ki hoặc integral_max
.Ki = 0.2f;
.integral_max = 100.0f;
```

---

**Tóm lại:** Integral term là giải pháp vì nó **tích lũy sai số theo thời gian**, buộc robot phải quay trở lại đến khi sai số = 0, không phải chỉ khi sai số hiện tại = 0.
