# TEST PLAN: Xác Minh Robot Tự Quay Trở Lại

## 🧪 Test 1: Kiểm Tra Cơ Bản - Đi Thẳng

### Mục Đích
Xác minh robot có tự quay trở lại khi đi thẳng

### Chuẩn Bị
- Robot trên sàn phẳng, không chướng ngại vật
- Camera hoặc GPS để theo dõi đường đi (nếu có)
- Serial monitor để xem debug output

### Thực Hiện
1. Bật robot
2. Để nó chạy thẳng 2-3 mét
3. Quan sát đường đi

### Kỳ Vọng (Sau Fix)
```
Thời điểm  Hành Động                      Yaw Angle    Status
────────────────────────────────────────────────────────────────
0s         Robot bắt đầu                  0°          ✓ Bình thường
1s         Phát hiện lệch nhẹ             +2°         ⚠ Bắt đầu lệch
2s         PID hiệu chỉnh → I term mạnh   +1.5°       🔧 Đang hiệu chỉnh
3s         Tiếp tục hiệu chỉnh            +0.5°       🔧 Tốc độ hội tụ nhanh
4s         Robot hội tụ về mục tiêu       0°          ✓ Thành công!
5s+        Duy trì                        ~0°±0.3°    ✓ Ổn định
```

**Điểm Khác Biệt:**
- **Trước:** Robot cứ đi lệch mãi, không quay trở lại
- **Sau:** Robot lệch, rồi tự quay trở lại trong vài giây

---

## 🧪 Test 2: Phản Ứng Sau Tác Động

### Mục Đích
Kiểm tra PID tự động khắc phục sau khi bị tác động

### Chuẩn Bị
- Robot đang chạy thẳng
- Có người hoặc vật để gõ nhẹ (không mạnh)

### Thực Hiện
1. Robot chạy thẳng trong 3 giây
2. Gõ nhẹ bề ngang (làm lệch ~5-10°)
3. Thả ra, quan sát robot tự hiệu chỉnh

### Kỳ Vọng
- Robot quay trở lại trong 1-2 giây
- Không còn tiếp tục lệch như trước

---

## 🧪 Test 3: Quan Sát Debug Output

### Mục Đích
Xác minh PID hoạt động từng thành phần P, I, D

### Chuẩn Bị
- Kết nối USB để xem Serial Monitor (115200 baud)

### Thực Hiện
1. Chạy robot
2. Mở Serial Monitor
3. Quan sát output mỗi 500ms:

```
[PID] Desired:0.00 Current:3.50 Error:-3.50 | P:-7.0 I:-1.2 D:-0.3 Correction:-8.5
      └─ Mục tiêu      └─ Hiện tại      └─ Sai số  └ P_term └ I_term └ D_term └ Tổng cộng
```

### Kỳ Vọng

**Column P (P-term):**
- Nếu error = -3.5°, P = Kp * error = 2.0 × (-3.5) = -7.0 ✓

**Column I (I-term):**
- Tăng dần khi error > 0 (tích lũy)
- Không giảm khi error nhỏ
- Ví dụ: -1.2 → -2.4 → -3.1 → -2.8 (giảm khi quay trở lại)

**Column D (D-term):**
- Nhỏ, biến thiên theo tốc độ thay đổi error
- Dương khi error giảm nhanh, âm khi tăng nhanh

**Column Correction:**
- Là tổng P + I + D
- Luôn "đẩy" robot về hướng mục tiêu

---

## 🧪 Test 4: Kiểm Tra Không Có Dao Động

### Mục Đích
Đảm bảo robot không zigzag (dao động quanh mục tiêu)

### Thực Hiện
1. Quan sát robot chạy 5-10 mét
2. Ghi lại: Có bao nhiêu lần quay trái-phải-trái?

### Kỳ Vọng (Đủ Khó)
- **Tốt:** 1-2 lần dao động nhỏ rồi ổn định
- **Không tốt:** Liên tục zigzag (lớn hơn ±5°)

**Nếu quá dao động:**
```c
// Tăng Kd (D-term làm mềm)
.Kd = 0.8f;  // Từ 0.5f
```

---

## 🧪 Test 5: Tuning Iterative

### Nếu Robot Vẫn Lệch

**Triệu chứng:** Robot không quay trở lại hoàn toàn

**Debug Check:**
```
Xem I-term có tăng liên tục không?
Nếu NOT → Ki quá thấp, TĂNG lên
Nếu YES → Có vấn đề khác (sensor, motor)
```

**Action:**
```c
// Tăng Ki để buộc robot quay về
.Ki = 0.4f;  // Từ 0.3f (tăng 33%)
// Test lại
```

### Nếu Robot Dao động Dã Man

**Triệu chứng:** Quay trái-phải-trái liên tục

**Debug Check:**
```
Xem Correction có âm-dương-âm liên tục không?
Nếu YES → Quá hiệu chỉnh, cần làm mềm
```

**Action:**
```c
// Tăng Kd (làm mềm)
.Kd = 0.7f;  // Từ 0.5f

// Hoặc giảm Kp (phản ứng kém)
.Kp = 1.5f;  // Từ 2.0f (giảm 25%)
```

---

## 📊 Bảng So Sánh: Trước vs Sau

| Tiêu Chí | Trước (P-only) | Sau (PID) |
|----------|----------------|----------|
| **Robot đi lệch?** | ✓ Có | ✓ Có (bình thường) |
| **Quay trở lại?** | ✗ KHÔNG | ✓ CÓ (tự động) |
| **Thời gian hội tụ** | ∞ (không bao giờ) | 2-5 giây |
| **Debug: P-term** | Giảm dần → 0 | Giảm dần → 0 |
| **Debug: I-term** | Không có | Tích lũy, buộc về |
| **Debug: D-term** | Không có | Làm mềm oscillation |
| **Kết quả** | Lệch vĩnh viễn | Ổn định trở lại |

---

## 🔧 Tham Số Để Thử

### Conservative (Chắc Chắn, Chậm)
```c
.Kp = 1.5f;
.Ki = 0.2f;
.Kd = 0.5f;
.integral_max = 150.0f;
```

### Balanced (Mặc Định, Cân Bằng)
```c
.Kp = 2.0f;
.Ki = 0.3f;
.Kd = 0.5f;
.integral_max = 150.0f;
```

### Aggressive (Nhanh, Có Thể Dao Động)
```c
.Kp = 2.5f;
.Ki = 0.5f;
.Kd = 0.7f;
.integral_max = 200.0f;
```

---

## 📝 Hướng Dẫn Điều Chỉnh Tham Số

### Bước 1: Kiểm Tra I-term Đang Hoạt Động
```
[PID] ... I:-1.2 ... → Tiếp tục tăng
[PID] ... I:-2.4 ... → Tích lũy ✓
[PID] ... I:-3.1 ... → Tích lũy ✓
```
Nếu I-term không thay đổi, Ki quá thấp → **TĂNG Ki**

### Bước 2: Kiểm Tra Dao Động
Nếu Correction giá trị nhảy cóc (5 → 0 → -5):
- **TĂNG Kd** để làm mềm
- Hoặc **GIẢM Ki** để tích lũy chậm hơn

### Bước 3: Kiểm Tra Phản Ứng Tốc Độ
Nếu robot phản ứng chậm (>5 giây mới quay lại):
- **TĂNG Kp** để phản ứng nhanh hơn
- Hoặc **TĂNG Ki** để tích lũy nhanh hơn

---

## ✅ Checklist Cuối Cùng

- [ ] Biên dịch code không lỗi
- [ ] Debug output hiển thị P/I/D term
- [ ] Robot không lệch khi đi thẳng
- [ ] Robot tự quay trở lại nếu bị lệch
- [ ] Không có dao động quá mức (±5°)
- [ ] Thời gian hội tụ < 5 giây
- [ ] Kiểm tra 3+ lần để chắc chắn

---

**Ghi chú:** Nếu vẫn có vấn đề, kiểm tra:
1. MPU6050 có hoạt động chính xác không?
2. Motor có cân bằng công suất không? (kiểm tra lệnh PWM 600/600 có đi thẳng?)
3. Debug output có in ra không?
