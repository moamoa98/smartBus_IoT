#include "esp_camera.h"
#include <WiFi.h>
#include <WebSocketsServer_Generic.h>  // Thư viện WebSocket
#include "base64.h"                    // Thư viện mã hóa Base64

// Chọn model camera
#define CAMERA_MODEL_AI_THINKER  // Model AI-Thinker
#include "camera_pins.h"

// Thông tin WiFi
const char* ssid = "TEKY OFFICE";
const char* password = "Teky@2018";

// Cổng WebSocket
#define WS_PORT 81
WebSocketsServer webSocket = WebSocketsServer(WS_PORT);

// Hàm khởi động camera
void startCamera() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;

  if (psramFound()) {
    config.frame_size = FRAMESIZE_QQVGA;  // 160x120 - Độ phân giải rất thấp
    config.jpeg_quality = 12;             // Chất lượng JPEG thấp hơn
    config.fb_count = 2;                  // Sử dụng 2 buffer
  } else {
    config.frame_size = FRAMESIZE_QQVGA;  // 160x120
    config.jpeg_quality = 15;             // Chất lượng JPEG thấp nhất
    config.fb_count = 1;                  // Sử dụng 1 buffer
  }


  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  sensor_t* s = esp_camera_sensor_get();
  s->set_framesize(s, FRAMESIZE_QQVGA);  // Giảm độ phân giải
  s->set_quality(s, 12);                 // Chất lượng JPEG thấp
  s->set_contrast(s, 1);                 // Tăng độ tương phản (tùy chọn)
  s->set_brightness(s, 0);               // Cân bằng sáng
  s->set_saturation(s, 0);               // Giảm bão hòa để xử lý nhanh hơn
}

// Hàm xử lý sự kiện WebSocket
void handleWebSocketEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t length) {
  switch (type) {
    case WStype_CONNECTED:
      Serial.printf("[%u] Connected!\n", num);
      break;
    case WStype_DISCONNECTED:
      Serial.printf("[%u] Disconnected!\n", num);
      break;
    default:
      break;
  }
}

void sendFrame() {
  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Failed to capture frame");
    return;
  }

  // Gửi dữ liệu nhị phân trực tiếp
  webSocket.broadcastBIN(fb->buf, fb->len);

  esp_camera_fb_return(fb);
}


void setup() {
  Serial.begin(115200);
  Serial.println();

  // Kết nối WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected");

  // Khởi động camera
  startCamera();

  // Khởi động WebSocket server
  webSocket.begin();
  webSocket.onEvent(handleWebSocketEvent);
  Serial.printf("WebSocket server started @ ws://%s:%d\n", WiFi.localIP().toString().c_str(), WS_PORT);
}

void loop() {
  webSocket.loop();  // Xử lý WebSocket
  sendFrame();       // Gửi frame ngay lập tức
  delay(320);         // ~60 FPS (1000ms / 60 ≈ 16ms)
}
