#include <CAN.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <string.h>

// ========================================================
// 1. CẤU HÌNH MẠNG WIFI VÀ ĐÍCH ĐẾN (CỦA LÂM)
// ========================================================
const char* ssid = "Lam";       
const char* password = "11111111";
const char* LAPTOP_IP = "172.20.10.2";     
const int UDP_PORT = 5005;                   

WiFiUDP udp;

// ========================================================
// 2. BIẾN LƯU TRỮ TRẠNG THÁI ODOMETRY
// ========================================================
float current_x = 0.0f;
float current_y = 0.0f;
float current_theta = 0.0f;
int16_t turn_dir = 0;  // 0: Thẳng, 90: Phải, -90: Trái, 180: Quay đầu
float turn_yaw = 0.0f;

void setup() {
  Serial.begin(115200);
  while (!Serial); 

  Serial.println("\n=====================================");
  Serial.println("  UGV GATEWAY: CAN TO WIFI (UDP)     ");
  Serial.println("=====================================");

  // --- KẾT NỐI WIFI ---
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.print("Dang ket noi WiFi.");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\n[OK] WiFi Da Ket Noi!");
  Serial.print("IP cua ESP32: ");
  Serial.println(WiFi.localIP());

  // --- KẾT NỐI CAN BUS ---
  CAN.setPins(4, 5); // Chân RX = 4, TX = 5
  
  if (!CAN.begin(500E3)) {
    Serial.println("[LOI] Khoi dong Module CAN That Bai!");
    while (1); 
  }
  Serial.println("[OK] Module CAN Da San Sang Lang Nghe!");
}

void loop() {
  int packetSize = CAN.parsePacket();
  
  if (packetSize) {
    long receivedId = CAN.packetId();
    uint8_t rx_data[8] = {0};
    int i = 0;

    // Đọc byte thô vào mảng
    while (CAN.available() && i < 8) {
      rx_data[i++] = CAN.read();
    }

    bool should_send_udp = false;

    // ==================================================
    // TRẠM 1: TỌA ĐỘ X, Y (ID: 0x201 - 8 byte)
    // ==================================================
    if (receivedId == 0x201 && packetSize == 8) {
      memcpy(&current_x, &rx_data[0], 4);
      memcpy(&current_y, &rx_data[4], 4);
      should_send_udp = true; 
    }
    
    // ==================================================
    // TRẠM 2: GÓC THETA (ID: 0x203 - 4 byte)
    // ==================================================
    else if (receivedId == 0x203 && packetSize == 4) {
      memcpy(&current_theta, &rx_data[0], 4);
      should_send_udp = true; 
    }

    // ==================================================
    // TRẠM 3: SỰ KIỆN RẼ CỦA NHẬT (ID: 0x202 - 6 byte)
    // ==================================================
    else if (receivedId == 0x202 && packetSize == 6) {
      memcpy(&turn_dir, &rx_data[0], 2);
      memcpy(&turn_yaw, &rx_data[2], 4);
      should_send_udp = true; 
    }

    // ==================================================
    // ĐÓNG GÓI JSON VÀ BẮN UDP LÊN LAPTOP
    // ==================================================
    if (should_send_udp) {
      char jsonStr[128]; 
      snprintf(jsonStr, sizeof(jsonStr), "{\"x\":%.3f,\"y\":%.3f,\"theta\":%.3f,\"turn\":%d}", 
               current_x, current_y, current_theta, turn_dir);

      udp.beginPacket(LAPTOP_IP, UDP_PORT);
      udp.print(jsonStr);
      udp.endPacket();
      
      // Xóa cờ turn_dir để tránh bị kẹt trạng thái
      if (turn_dir != 0) turn_dir = 0;
      
      // In ra Serial Monitor để kiểm tra (Lâm để dòng này lại nhé)
      Serial.println(jsonStr); 
    }
  }
}