#include <Arduino.h>
#include <ESP32Servo.h>

#define LED 0
#define INA 15
#define INB 2
#define SERVO_PIN 4

#define SAFE_DISTANCE  30    // Khoảng cách an toàn (cm)
#define TURN_ANGLE  40      // Góc quay servo (độ)
#define CENTER_ANGLE  90     // Góc trung tâm servo

#define TURN_DELAY  1000     // Thời gian để xe quay (ms)
#define REVERSE_TIME 1500 

//ultra
#define TRIG_RIGHT 35
#define ECHO_RIGHT 34
#define TRIG_FRONT 32
#define ECHO_FRONT 33
#define TRIG_LEFT 26
#define ECHO_LEFT 25

//tạo đối tượng servo
Servo steeringServo;

void setSteeringAngle(int angle) {
  steeringServo.write(angle);
  delay(200);  // Đợi servo quay đến vị trí
}

int getDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  long duration = pulseIn(echoPin, HIGH);
  return duration * 0.034 / 2;
}

// Các hàm điều khiển động cơ đơn giản
void stopMotor() {
  digitalWrite(INA, LOW);
  digitalWrite(INB, LOW);
}

void forward() {
  digitalWrite(INA, LOW);
  digitalWrite(INB, HIGH);
}

void backward() {
  digitalWrite(INA, HIGH);
  digitalWrite(INB, LOW);
}

void reverseAndTurn() {
  // Dừng xe
  stopMotor();
  delay(200);
  
  // Lùi lại
  Serial.println("Reversing");
  setSteeringAngle(CENTER_ANGLE);
  backward();
  delay(REVERSE_TIME);
  
  // Dừng lại sau khi lùi
  stopMotor();
  delay(200);
  
  // Quay sang phải
  Serial.println("Turning Right after reverse");
  setSteeringAngle(CENTER_ANGLE + TURN_ANGLE);
  forward();
  delay(TURN_DELAY * 1.5);
  
  // Đưa bánh xe về giữa
  setSteeringAngle(CENTER_ANGLE);
}

void setup() {
  Serial.begin(115200);
  
  //động cơ dc
  pinMode(INA, OUTPUT);
  pinMode(INB, OUTPUT);
  
  //ultrasonicSensor
  pinMode(TRIG_RIGHT, OUTPUT);
  pinMode(ECHO_RIGHT, INPUT);
  pinMode(TRIG_FRONT, OUTPUT);
  pinMode(ECHO_FRONT, INPUT);
  pinMode(TRIG_LEFT, OUTPUT);
  pinMode(ECHO_LEFT, INPUT);

  steeringServo.attach(SERVO_PIN);
  setSteeringAngle(CENTER_ANGLE);

  // Khởi động với trạng thái dừng
  stopMotor();
}

void loop() {
  // Đọc khoảng cách từ các cảm biến
  // int distanceFront = getDistance(TRIG_FRONT, ECHO_FRONT);
  // int distanceLeft = getDistance(TRIG_LEFT, ECHO_LEFT);
  // int distanceRight = getDistance(TRIG_RIGHT, ECHO_RIGHT);
  
  // Serial.printf("Distance - Front: %d cm, Left: %d cm, Right: %d cm\n", 
  //               distanceFront, distanceLeft, distanceRight);
  
  // if (distanceFront > SAFE_DISTANCE) {
  //   // Đường phía trước an toàn, đi thẳng
  //   setSteeringAngle(CENTER_ANGLE);
  //   forward();
  // } else {
  //   // Dừng xe để kiểm tra hướng
  //   stopMotor();
  //   delay(500);
    
  //   if (distanceLeft > distanceRight && distanceLeft > SAFE_DISTANCE) {
  //     // Rẽ trái nếu bên trái có nhiều không gian hơn
  //     Serial.println("Turning Left");
  //     setSteeringAngle(CENTER_ANGLE - TURN_ANGLE);
  //     forward();
  //     delay(TURN_DELAY);
  //   } else if (distanceRight > SAFE_DISTANCE) {
  //     // Rẽ phải nếu bên phải có không gian
  //     Serial.println("Turning Right");
  //     setSteeringAngle(CENTER_ANGLE + TURN_ANGLE);
  //     forward();
  //     delay(TURN_DELAY);
  //   } else {
  //     // Không có đường đi, lùi lại và quay đầu
  //     reverseAndTurn();
  //   }
  // }

  forward();
  // delay(2000);
  // backward();
  // delay(2000);

}