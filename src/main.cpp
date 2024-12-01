#include <Arduino.h>
#include <NewPing.h>
#include <ESP32Servo.h>
#include <WiFi.h>
#include <WebServer.h>

//wifi setup
const char* ssid = "TEKY OFFICE";      
const char* password = "Teky@2018"; 

WebServer server(80);

String serialData = "";

// Pin định nghĩa
#define TRIG_LEFT 14
#define ECHO_LEFT 27
#define TRIG_FRONT 26
#define ECHO_FRONT 25
#define TRIG_RIGHT 33
#define ECHO_RIGHT 32

#define SERVO_PIN 13
#define INA 16
#define INB 17

// Các hằng số
#define SAFE_DISTANCE 30    // Khoảng cách an toàn (cm)
#define CENTER_ANGLE 90     // Góc trung tâm servo
#define MAX_ANGLE 130       // Góc tối đa bên phải
#define MIN_ANGLE 50        // Góc tối đa bên trái
#define MOTOR_SPEED 150     // Tốc độ động cơ
#define TURN_DELAY 1000     // Thời gian quay

//kích thước 5 mảng 5 lần xét gần nhất
#define FILTER_SIZE 5 

SemaphoreHandle_t distanceMutex;

enum Direction { NONE, LEFT, RIGHT };
Direction currentDirection = NONE;

int leftDistances[FILTER_SIZE] = {0};
int frontDistances[FILTER_SIZE] = {0};
int rightDistances[FILTER_SIZE] = {0};

volatile int distanceLeft=0;
volatile int distanceFront=0;
volatile int distanceRight=0;

int leftIndex = 0, frontIndex = 0, rightIndex = 0;

Servo servo;

// Cấu hình cảm biến siêu âm
NewPing sensorLeft(TRIG_LEFT, ECHO_LEFT, 400);
NewPing sensorFront(TRIG_FRONT, ECHO_FRONT, 400);
NewPing sensorRight(TRIG_RIGHT, ECHO_RIGHT, 400);

// Hàm đọc khoảng cách
// int getDistance(NewPing sensor) {
//   delay(50);
//   return sensor.ping_cm();
// }
int getFilteredDistance(NewPing sensor, int* distanceArray, int& currentIndex);
void setMotorSpeed(int speed, bool forward);
void adjustServo(int angle);

void readDistanceTask(void *pvParameters) {


  while (true) {

    int Left = getFilteredDistance(sensorLeft, leftDistances, leftIndex);
    int Front = getFilteredDistance(sensorFront, frontDistances, frontIndex);
    int Right = getFilteredDistance(sensorRight, rightDistances, rightIndex);
    
    Serial.print("Left: "); Serial.print(distanceLeft);
    Serial.print(" Front: "); Serial.print(distanceFront);
    Serial.print(" Right: "); Serial.println(distanceRight);

    //serialData = "Left:"+String(distanceLeft)+","+ "Font:"+String(distanceFront)+","+ "Right:"+String(distanceRight)+"\n";
    //Serial.println(serialData);
   // server.handleClient(); // xử lý yêu cầu từ client

   if(xSemaphoreTake(distanceMutex,portMAX_DELAY)==pdTRUE){
      distanceLeft=Left;
      distanceFront=Front;
      distanceRight=Right;
      xSemaphoreGive(distanceMutex);
   }

   vTaskDelay(pdMS_TO_TICKS(100));
  }
}

void navigationTask(void *Parameters){

  while (true)
  {
    int LocalDistanceLeft,LocalDistanceFront,LocalDistanceRight;

    if(xSemaphoreTake(distanceMutex,portMAX_DELAY)==pdTRUE){
      LocalDistanceLeft=distanceLeft;
      LocalDistanceFront=distanceFront;
      LocalDistanceRight=distanceRight;
      xSemaphoreGive(distanceMutex);
   }

  if ((distanceFront < SAFE_DISTANCE && distanceFront > 0) || 
    (distanceLeft < SAFE_DISTANCE && distanceLeft > 0) || 
    (distanceRight < SAFE_DISTANCE && distanceRight > 0)) {
    // Dựa trên trạng thái để quay trái/phải
    if (currentDirection == NONE) {
        // Ưu tiên đi về phía có khoảng cách lớn hơn
        if (distanceLeft > distanceRight && distanceLeft > distanceFront) {
            currentDirection = LEFT;
        } else if (distanceRight > distanceLeft && distanceRight > distanceFront) {
            currentDirection = RIGHT;
        } else {
            // Nếu cả 3 hướng đều chật, quay ngược lại
            currentDirection = (random(2) == 0) ? LEFT : RIGHT;
        }
    }
    
    if (currentDirection == LEFT) {
        adjustServo(MIN_ANGLE); // Quay trái
    } else if (currentDirection == RIGHT) {
        adjustServo(MAX_ANGLE); // Quay phải
    }
    
    setMotorSpeed(MOTOR_SPEED, false); // Lùi lại một chút
    delay(TURN_DELAY);
} else {
    // Trở về trạng thái cân bằng nếu không có vật cản
    adjustServo(CENTER_ANGLE);
    setMotorSpeed(MOTOR_SPEED, true); // Tiếp tục tiến
    currentDirection = NONE; // Reset trạng thái
}

    vTaskDelay(pdMS_TO_TICKS(50));
  }
  
}

int getFilteredDistance(NewPing sensor, int* distanceArray, int& currentIndex) {
  // Đọc khoảng cách mới
  int newDistance = sensor.ping_cm();
  
  // Lưu giá trị mới vào mảng
  distanceArray[currentIndex] = newDistance;
  
  // Tăng chỉ số, quay lại 0 nếu vượt quá kích thước mảng
  currentIndex = (currentIndex + 1) % FILTER_SIZE;
  
  // Tính trung bình
  long sum = 0;
  int validMeasurements = 0;
  
  for (int i = 0; i < FILTER_SIZE; i++) {
    // Chỉ tính những giá trị khác 0 (loại bỏ các phép đo không hợp lệ)
    if (distanceArray[i] > 0) {
      sum += distanceArray[i];
      validMeasurements++;
    }
  }
  
  // Trả về 0 nếu không có phép đo hợp lệ
  return (validMeasurements > 0) ? sum / validMeasurements : 0;
}

// Hàm điều khiển động cơ
void setMotorSpeed(int speed, bool forward) {
  if (forward) {
      analogWrite(INA, speed);
      analogWrite(INB, 0);
  } else {
     analogWrite(INA, 0);
     analogWrite(INB, speed);
  }
}

// Điều chỉnh servo
void adjustServo(int angle) {
  angle = constrain(angle, MIN_ANGLE, MAX_ANGLE); // Giới hạn góc
  servo.write(angle);
}

// Trạng thái hướng quay


void setup() {
  Serial.begin(115200);

  WiFi.begin(ssid, password);

  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

    Serial.println("Connected!");

  // In địa chỉ IP của ESP32
  Serial.println(WiFi.localIP());

  // Cấu hình Web Server
  server.on("/", []() {
    server.send(200, "text/plain", serialData);
  });

  server.begin();
  Serial.println("Web Server started");
  
  // Cấu hình pin
  pinMode(INA, OUTPUT);
  pinMode(INB, OUTPUT);
  
  servo.attach(SERVO_PIN);
  servo.write(CENTER_ANGLE);

   

  distanceMutex=xSemaphoreCreateMutex();

  xTaskCreate(
    readDistanceTask,     // Hàm task
    "ReadDistance",       // Tên task
    2048,                 // Kích thước stack
    NULL,                 // Tham số truyền vào (không có)
    1,                    // Mức ưu tiên
    NULL                  // Handle task (không cần)
  );

  xTaskCreate(
    navigationTask,       // Hàm task
    "Navigation",         // Tên task
    2048,                 // Kích thước stack
    NULL,                 // Tham số truyền vào (không có)
    1,                    // Mức ưu tiên
    NULL                  // Handle task (không cần)
  );
}

void loop() {
  // int distanceLeft = getDistance(sensorLeft);
  // int distanceFront = getDistance(sensorFront);
  // int distanceRight = getDistance(sensorRight);

  // int distanceLeft = getFilteredDistance(sensorLeft, leftDistances, leftIndex);
  // int distanceFront = getFilteredDistance(sensorFront, frontDistances, frontIndex);
  // int distanceRight = getFilteredDistance(sensorRight, rightDistances, rightIndex);
  
  // Serial.print("Left: "); Serial.print(distanceLeft);
  // Serial.print(" Front: "); Serial.print(distanceFront);
  // Serial.print(" Right: "); Serial.println(distanceRight);

  // serialData = "Left:"+String(distanceLeft)+","+ "Font:"+String(distanceFront)+","+ "Right:"+String(distanceRight)+"\n";
  // Serial.println(serialData);
  // server.handleClient(); // xử lý yêu cầu từ client

  // Phát hiện vật cản phía trước
  // if (distanceFront < SAFE_DISTANCE && distanceFront > 0) {
  //   // Dựa trên trạng thái để quay trái/phải
  //   if (currentDirection == NONE) {
  //     if (distanceLeft > distanceRight) {
  //       currentDirection = LEFT;
  //     } else {
  //       currentDirection = RIGHT;
  //     }
  //   }

  //   if (currentDirection == LEFT) {
  //     adjustServo(MIN_ANGLE); // Quay trái
  //   } else if (currentDirection == RIGHT) {
  //     adjustServo(MAX_ANGLE); // Quay phải
  //   }

  //   setMotorSpeed(MOTOR_SPEED, false); // Lùi lại một chút
  //   delay(TURN_DELAY);
  // } else {
  //   // Trở về trạng thái cân bằng nếu không có vật cản
  //   adjustServo(CENTER_ANGLE);
  //   setMotorSpeed(MOTOR_SPEED, true); // Tiếp tục tiến
  //   currentDirection = NONE; // Reset trạng thái
  // }
  // delay(100);

  vTaskDelete(NULL);
}
