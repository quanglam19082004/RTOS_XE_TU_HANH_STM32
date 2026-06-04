# Hướng Dẫn PID Điều Chỉnh Yaw (Hướng Đi Robot)

## 📋 Vấn Đề Gốc Rễ (Root Cause)
Robot đi lệch khi chạy thẳng và **không tự quay trở lại đường thẳng ban đầu** vì:
- **Cũ:** Chỉ dùng bộ điều khiển `P (Proportional)` → Khi sai số → 0, hiệu chỉnh cũng → 0
- **Kết quả:** Robot chỉ giảm bớt độ lệch nhưng không bao giờ quay trở lại mục tiêu

## 🔧 Giải Pháp: PID Controller

### Cơ Chế Hoạt Động

```
Correction = Kp × Error + Ki × ∫Error + Kd × (dError/dt)
              └─ P term ─┘   └─ I term ──┘  └─ D term ──┘
                Quick response  Eliminates    Smoothing
                to deviation   steady-state  & damping
                               error
```

### Ba Thành Phần PID:

| Thành Phần | Công Dụng | Vấn Đề Nếu Quá Cao | Vấn Đề Nếu Quá Thấp |
|-----------|----------|------------------|-------------------|
| **P (Kp)** | Phản ứng nhanh với sai số hiện tại | Dao động quanh mục tiêu | Phản ứng chậm |
| **I (Ki)** | **QUAN TRỌNG**: Tích lũy sai số để buộc robot về mục tiêu | Tích lũy quá mức (windup), dao động dã man | Robot không quay trở lại, tiếp tục lệch |
| **D (Kd)** | Mềm mại hiệu chỉnh, giảm dao động | Quá cục bộ, phản ứng chậm trên thay đổi nhanh | Có thể dao động |

## ⚙️ Tham Số Hiện Tại (Current Tuning)

```c
.Kp = 2.0f                    // Gain tỷ lệ (phản ứng nhanh)
.Ki = 0.3f                    // Gain tích phân (KỲ CỤC - ĐÂY GIẢI QUYẾT VẤN ĐỀ)
.Kd = 0.5f                    // Gain đạo hàm (làm mền mại)
.integral_max = 150.0f        // Anti-windup limit (tránh tích lũy quá mức)
.output_max = 400.0f          // Max correction output
```

## 🎯 Cách Điều Chỉnh Tuning

### **Bước 1: Kiểm Tra Vấn Đề Hiện Tại**
Chạy robot thẳng 2-3 mét và quan sát:
- ✅ Robot quay trở lại mục tiêu (sau khi lệch)?
- ✅ Có dao động quanh đường thẳng?
- ✅ Tốc độ hội tụ (chảy về mục tiêu)?

### **Bước 2: Điều Chỉnh Kp (Proportional)**
```c
// Nếu robot phản ứng chậm:
.Kp = 3.0f;   // Tăng (nhưng có thể gây dao động)

// Nếu robot dao động dã man quanh mục tiêu:
.Kp = 1.5f;   // Giảm
```

### **Bước 3: Điều Chỉnh Ki (Integral) - QUAN TRỌNG**
```c
// Nếu robot vẫn lệch sau hiệu chỉnh:
.Ki = 0.5f;   // TĂNG (để tăng cường tích phân)
// Điều này BẮTBUỘC robot về đúng yaw, kể cả khi có ma sát bất đối xứng

// Nếu robot dao động hoặc quá hiệu chỉnh:
.Ki = 0.2f;   // Giảm
```

### **Bước 4: Điều Chỉnh Kd (Derivative)**
```c
// Nếu hiệu chỉnh quá dao động:
.Kd = 0.8f;   // TĂNG (để làm mềm)

// Nếu hiệu chỉnh quá "lơ là":
.Kd = 0.3f;   // Giảm
```

### **Bước 5: Điều Chỉnh Anti-Windup**
```c
// Nếu tích phân quá tích lũy (robot quay liên tục):
.integral_max = 100.0f;   // GIẢM giới hạn tích lũy

// Nếu robot không đủ mạnh để quay trở lại:
.integral_max = 200.0f;   // TĂNG giới hạn
```

## 📊 Quy Trình Điều Chỉnh Đề Xuất

1. **Bước 1:** Chạy với tham số mặc định, quan sát hành vi
2. **Bước 2:** Nếu robot không quay trở lại → TĂNG `Ki` (tích phân)
3. **Bước 3:** Nếu quá dao động → TĂNG `Kd` (để mềm mại)
4. **Bước 4:** Nếu phản ứng chậm → TĂNG `Kp`
5. **Bước 5:** Lặp lại cho đến khi hài lòng

## 🔍 Quan Sát Debug Output

```
[PID] Desired:0.00 Current:2.34 Error:-2.34 | P:-4.7 I:-3.1 D:-0.2 Correction:-8.0 | Cmd_L:592 Cmd_R:608
      └──────┬──────┘ └────┬────┘ └──┬──┘   └──────────────────────────────────────────────────────────┘
         Target    Current   Sai số    Đóng góp từ từng thành phần (P/I/D)
```

- **Desired vs Current:** Sự khác biệt là bao nhiêu?
- **Error:** Sai lệch yaw (độ)
- **P/I/D:** Đóng góp từng thành phần vào hiệu chỉnh
- **Cmd_L/Cmd_R:** Lệnh PWM gửi đến motor

## ⚠️ Nguyên Tắc Điều Chỉnh

| Vấn Đề | Giải Pháp |
|--------|----------|
| Robot không quay trở lại mục tiêu | **TĂNG Ki** (tích phân là then chốt) |
| Robot dao động liên tục | **TĂNG Kd**, giảm Kp |
| Robot chậm đáp ứng | **TĂNG Kp** |
| Quá hiệu chỉnh (zigzag) | **GIẢM Kp**, tăng Kd |
| Motor bị "kẹt" quay | Giảm Ki, tăng anti-windup |

## 📌 Ghi Chú Quan Trọng

1. **Ki là chìa khóa:** Nó buộc robot phải quay trở lại, kể cả khi motor không đối xứng
2. **Anti-windup:** Ngăn tích phân tích lũy vô hạn khi sai số không về 0
3. **Thử nghiệm từng thông số một:** Đừng thay đổi tất cả cùng lúc
4. **Debug output** sẽ in ra từ 500ms lần để bạn theo dõi

## 🧪 Test Cases

### Test 1: Đi Thẳng
```
1. Bắt đầu robot, để nó đi thẳng 2-3 mét
2. Quan sát: Có quay trở lại hay tiếp tục lệch?
3. Kỳ vọng: Robot sẽ nhỏ lệch ngoài phút đầu, rồi tự quay trở lại
```

### Test 2: Hiệu Chỉnh Sau Tác Động
```
1. Robot đang chạy
2. Lật nhẹ bằng tay (hoặc gõ nhẹ) để tạo lệch
3. Kỳ vọng: Robot tự quay trở lại, không tiếp tục lệch
```

### Test 3: Dao Động
```
1. Quan sát xem robot có zigzag hay không
2. Nếu có: Tăng Kd (D term làm mềm)
```

---

**Chúc bạn thành công! Nếu vẫn gặp vấn đề, hãy kiểm tra debug output để thấy đóng góp từng thành phần P/I/D.**
