Mobile Robot Odometry Tracking System based on FreeRTOS & UDP
1. Giới thiệu dự án
Mobile Robot Odometry Tracking System là dự án xây dựng hệ thống xe tự hành hai bánh độc lập (Differential Drive). Hệ thống sử dụng hệ điều hành thời gian thực FreeRTOS để quản lý đa tác vụ, tính toán động học xe và vẽ bản đồ thời gian thực (real-time odometry mapping).

Dự án là sự kết hợp hoàn chỉnh giữa lập trình vi điều khiển, giao tiếp không dây và xây dựng phần mềm giám sát trên máy tính (Ground Station), thể hiện tư duy thiết kế hệ thống nhúng toàn diện (Full-stack Embedded/Robotics).
2. Phần cứng hệ thống
Hệ thống sử dụng các thành phần phần cứng sau:

Vi điều khiển trung tâm: STM32F411CEU6.
Module WiFi: ESP32.
Giao tiếp: Giao tiếp CAN giữa STM32 và ESP32.
Mạch điều khiển động cơ: L298N.
Cảm biến: Encoder v1.
3. Kiến trúc phần mềm & Giao thức truyền thông
Dự án được chia thành 3 khối chính, lưu trữ trong cùng một repository:

RTOS_XE_TU_HANH: Firmware điều khiển xe (C/C++ - STM32CubeIDE).
Sử dụng FreeRTOS để quản lý các tác vụ như: đọc cảm biến encoder, điều khiển động cơ PID, và tính toán động học (Odometry).
link_wifi_udp: Firmware cầu nối không dây (C/C++ - Arduino IDE).
Chạy trên ESP32, làm nhiệm vụ nhận dữ liệu từ STM32 qua CAN bus và gửi không dây về trạm mặt đất.
Ground_Station: Phần mềm trạm mặt đất (Python).
Nhận dữ liệu qua giao thức UDP và trực quan hóa quỹ đạo di chuyển của xe trên giao diện đồ họa.

Luồng dữ liệu (Data Flow): STM32F411CEU6 $\rightarrow$ [CAN Bus] $\rightarrow$ ESP32 $\rightarrow$ [UDP over WiFi] $\rightarrow$ Ground Station (Python).

Giao thức truyền thông (UDP Payload):

ESP32 đóng gói dữ liệu trạng thái của robot thành một chuỗi JSON (String) và gửi qua UDP.
Cấu trúc JSON có dạng: {"x": <giá trị x>, "y": <giá trị y>, "theta": <giá trị góc theta>, "turn": <trạng thái rẽ>}.
4. Yêu cầu cài đặt (Dependencies)
Ứng dụng Ground_Station được viết bằng Python. Để chạy được giao diện vẽ bản đồ, bạn cần cài đặt các thư viện sau:

socket
json
threading
matplotlib
math
5. Hướng dẫn sử dụng
Cấu hình mạng WiFi (SSID và Mật khẩu) trong firmware của ESP32 (thư mục link_wifi_udp). Nạp firmware cho ESP32 và STM32.
Cấp nguồn cho xe tự hành.
Đảm bảo máy tính chạy Ground_Station đang kết nối cùng mạng WiFi với ESP32.
Mở terminal, di chuyển đến thư mục Ground_Station.
Chạy lệnh: python robot_map.py.
Giao diện "BẢN ĐỒ ODOMETRY TUYỆT ĐỐI" sẽ hiện ra và cập nhật liên tục quỹ đạo của xe.
## 6. Kết quả thực tế

Dưới đây là hình ảnh phần cứng thực tế của robot và các bản đồ quỹ đạo (odometry) được thu thập từ Ground Station.

### 🤖 Mô hình phần cứng xe tự hành
![Robot tự hành](./Screenshot%202026-10-18%20112604.jpg)

### 🗺️ Bản đồ quỹ đạo Odometry
![Quỹ đạo xe chạy](./Map_pic_2.png)

### 🎬 Video Demo thực tế
<video src="./3.mp4" controls width="100%"></video>
*(Nếu video không tự chạy trực tiếp trên trình duyệt do dung lượng lớn, bạn có thể bấm vào repo để tải file `3.mp4` về xem).*
