import socket
import json
import threading
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import math

# ==========================================
# CẤU HÌNH MẠNG UDP
# ==========================================
UDP_IP = "0.0.0.0"
UDP_PORT = 5005

# ==========================================
# BIẾN TOÀN CỤC LƯU TRỮ DỮ LIỆU
# ==========================================
real_x_data = [0.0]
real_y_data = [0.0]

# Tọa độ và Góc hướng TUYỆT ĐỐI của bản đồ
abs_x = 0.0
abs_y = 0.0
abs_theta = 0.0  

# Bộ đệm để tính quãng đường di chuyển mỗi chu kỳ
last_raw_x = 0.0
last_raw_y = 0.0

current_theta_draw = 0.0

# ==========================================
# LUỒNG CHẠY NGẦM: LẮNG NGHE UDP VÀ TÍNH TOÁN MA TRẬN XOAY
# ==========================================
def udp_listener():
    global abs_x, abs_y, abs_theta, last_raw_x, last_raw_y, current_theta_draw
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP, UDP_PORT))
    print(f"[OK] Đang lắng nghe UDP trên cổng {UDP_PORT}...")
    
    while True:
        try:
            data, addr = sock.recvfrom(1024)
            message = data.decode('utf-8').strip()
            robot_state = json.loads(message)
            
            x = robot_state.get("x", 0.0)
            y = robot_state.get("y", 0.0)
            theta = robot_state.get("theta", 0.0)
            turn = robot_state.get("turn", 0)

            # =======================================================
            # BỘ LỌC CHỐNG NHIỄU (SẬP NGUỒN/RESET)
            # Nếu tọa độ nhảy vọt quá 1 mét trong chớp mắt (STM32 bị reset)
            # Ta bỏ qua nó và cấu hình lại mốc đo.
            # =======================================================
            if math.hypot(x - last_raw_x, y - last_raw_y) > 1.0:
                last_raw_x = x
                last_raw_y = y
                continue 

            # =======================================================
            # CẬP NHẬT GÓC HƯỚNG KHI CÓ SỰ KIỆN RẼ
            # Quy ước của bạn: -90 là Trái, 90 là Phải.
            # =======================================================
            if turn != 0:
                abs_theta -= math.radians(turn)
                print(f"[SỰ KIỆN] Xe bẻ lái {turn} độ. Hướng trên bản đồ: {math.degrees(abs_theta):.0f} độ")

            # Tính quãng đường (delta) mà xe vừa nhích thêm
            dx_raw = x - last_raw_x
            dy_raw = y - last_raw_y
            
            last_raw_x = x
            last_raw_y = y

            # Chiếu quãng đường này lên bản đồ bằng ma trận xoay 2D
            dx_real = dx_raw * math.cos(abs_theta) - dy_raw * math.sin(abs_theta)
            dy_real = dx_raw * math.sin(abs_theta) + dy_raw * math.cos(abs_theta)

            abs_x += dx_real
            abs_y += dy_real

            # Chỉ lưu điểm nếu xe thực sự di chuyển (tránh nặng máy)
            if abs(dx_real) > 0.001 or abs(dy_real) > 0.001 or turn != 0:
                real_x_data.append(abs_x)
                real_y_data.append(abs_y)
            
            # Góc vẽ mũi tên = Góc tuyệt đối của bản đồ + Góc nghiêng lắc nhẹ của xe hiện tại
            current_theta_draw = abs_theta + theta
                
        except json.JSONDecodeError:
            pass
        except Exception:
            pass

# ==========================================
# LUỒNG CHÍNH: VẼ ĐỒ THỊ REAL-TIME
# ==========================================
fig, ax = plt.subplots(figsize=(10, 8))
line, = ax.plot([], [], 'b-', linewidth=2.5, label='Quỹ đạo xe chạy')
arrow = None

def update_plot(frame):
    global arrow
    if len(real_x_data) < 2:
        return line,
        
    line.set_data(real_x_data, real_y_data)
    
    # Tự động co giãn khung hình, cho phép vẽ qua các vùng âm
    margin = 0.5 # Mở rộng lề 0.5 mét
    ax.set_xlim(min(real_x_data) - margin, max(real_x_data) + margin)
    ax.set_ylim(min(real_y_data) - margin, max(real_y_data) + margin)
    
    if arrow:
        arrow.remove()
        
    last_x = real_x_data[-1]
    last_y = real_y_data[-1]
    
    # Chiều dài và hướng của mũi tên đỏ
    arrow_dx = 0.3 * math.cos(current_theta_draw)
    arrow_dy = 0.3 * math.sin(current_theta_draw)
    
    arrow = ax.arrow(last_x, last_y, arrow_dx, arrow_dy, 
                     head_width=0.08, head_length=0.1, 
                     fc='red', ec='red', zorder=10)
    return line,

if __name__ == '__main__':
    thread = threading.Thread(target=udp_listener, daemon=True)
    thread.start()
    
    ax.set_title("BẢN ĐỒ ODOMETRY TUYỆT ĐỐI", fontsize=14, fontweight='bold')
    ax.set_xlabel("Trục X (dm)")
    ax.set_ylabel("Trục Y (dm)")
    ax.grid(True, linestyle='--', alpha=0.7)
    
    # Khóa tỉ lệ X, Y bằng nhau để xe rẽ đúng góc vuông, không bị méo map
    ax.set_aspect('equal', adjustable='datalim')
    ax.legend()
    
    ani = animation.FuncAnimation(fig, update_plot, interval=100, cache_frame_data=False)
    
    plt.tight_layout()
    plt.show()