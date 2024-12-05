#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include "SPIFFS.h"
#include <ESP32Servo.h>
#include <NewPing.h>

int currentDegree = 90;
int maxLeft = 50;
int maxRight = 140;
Servo servo;

#define SERVO_PIN 13
#define INA 16
#define INB 17


/*
  The resolution of the PWM is 8 bit so the value is between 0-255
  We will set the speed between 100 to 255.
*/
enum speedSettings
{
  SLOW = 100,
  NORMAL = 180,
  FAST = 255
};

// tạo đối tượng servo
Servo steeringServo;

void setSteeringAngle(int angle)
{
  steeringServo.write(angle);
}

class Car
{
private:

  // PWM Setup to control motor speed
  // const int SPEED_CONTROL_PIN = 16;

  // Play around with the frequency settings depending on the motor that you are using
  const int freq = 2000;
  const int channel = 1;
  // 8 Bit resolution for duty cycle so value is between 0 - 255
  const int resolution = 8;

  // holds the current speed settings, see values for SLOW, NORMAL, FAST
  speedSettings currentSpeedSettings;

public:
  Car()
  {
    // Set all pins to output
    pinMode(INA, OUTPUT);
    pinMode(INB, OUTPUT);
    // pinMode(SPEED_CONTROL_PIN, OUTPUT);

    // Set initial motor state to OFF
    digitalWrite(INA, LOW);
    digitalWrite(INB, LOW);

    // Set the PWM Settings
    ledcSetup(channel, freq, resolution);

    // // Attach Pin to Channel
    // ledcAttachPin(SPEED_CONTROL_PIN, channel);

    // initialize default speed to SLOW
    setCurrentSpeed(speedSettings::NORMAL);
  }

  // Turn the car left

  // Move the car forward
  void moveForward()
  {
    Serial.println("car is moving forward...");
    setMotorSpeed();
    // digitalWrite(INB, LOW);
    // digitalWrite(INA, HIGH);
    analogWrite(INA, currentSpeedSettings);
    analogWrite(INB, 0);
  }

  // Move the car backward
  void moveBackward()
  {
    setMotorSpeed();
    Serial.println("car is moving backward...");
    // digitalWrite(INB, HIGH);
    // digitalWrite(INA, LOW);
    analogWrite(INA, 0);
    analogWrite(INB, currentSpeedSettings);
  }

  // Stop the car
  void stop()
  {
    Serial.println("car is stopping...");
    ledcWrite(channel, 0);

    // // Turn off motors
    // digitalWrite(INA, LOW);
    // digitalWrite(INB, LOW);
    analogWrite(INA, 0);
    analogWrite(INB, 0);
  }

  void turnRight()
  {
    Serial.println("car is turn left...");
    ledcWrite(channel, 0);
    setSteeringAngle(maxRight);
  }
  void turnLeft()
  {
    Serial.println("car is turn right...");
    ledcWrite(channel, 0);
    setSteeringAngle(maxLeft);
  }

  // Set the motor speed
  void setMotorSpeed()
  {
    // change the duty cycle of the speed control pin connected to the motor
    Serial.print("Speed Settings: ");
    Serial.println(currentSpeedSettings);
    ledcWrite(channel, currentSpeedSettings);
  }
  // Set the current speed
  void setCurrentSpeed(speedSettings newSpeedSettings)
  {
    Serial.println("car is changing speed...");
    currentSpeedSettings = newSpeedSettings;
  }
  // Get the current speed
  speedSettings getCurrentSpeed()
  {
    return currentSpeedSettings;
  }
};

// Change this to your network SSID
const char *ssid = "TEKY OFFICE";
const char *password = "Teky@2018";

// AsyncWebserver runs on port 80 and the asyncwebsocket is initialize at this point also
AsyncWebServer server(80);
AsyncWebSocket carWs("/carWs");
AsyncWebSocket servoWs("/servoWs");

// Our car object
Car car;

// Pin definitions for autorun
#define TRIG_LEFT 14
#define ECHO_LEFT 27
#define TRIG_FRONT 26
#define ECHO_FRONT 25
#define TRIG_RIGHT 33
#define ECHO_RIGHT 32

// Constants for autorun
#define SAFE_DISTANCE 30
#define CENTER_ANGLE 90
#define MAX_ANGLE 130
#define MIN_ANGLE 50
#define MOTOR_SPEED 150
#define TURN_DELAY 1000
#define FILTER_SIZE 5

// Autorun variables
SemaphoreHandle_t distanceMutex;
enum Direction { NONE, LEFT, RIGHT };
Direction currentDirection = NONE;
enum ControlMode { AUTO, MANUAL };
volatile ControlMode currentControlMode = MANUAL;
TaskHandle_t readDistanceTaskHandle = NULL;
TaskHandle_t navigationTaskHandle = NULL;
int leftDistances[FILTER_SIZE] = {0};
int frontDistances[FILTER_SIZE] = {0};
int rightDistances[FILTER_SIZE] = {0};
volatile int distanceLeft = 0;
volatile int distanceFront = 0;
volatile int distanceRight = 0;
int leftIndex = 0, frontIndex = 0, rightIndex = 0;
NewPing sensorLeft(TRIG_LEFT, ECHO_LEFT, 400);
NewPing sensorFront(TRIG_FRONT, ECHO_FRONT, 400);
NewPing sensorRight(TRIG_RIGHT, ECHO_RIGHT, 400);

void switchControlMode(ControlMode);
int getFilteredDistance(NewPing, int *, int &);
void setMotorSpeed(int, bool);
// Function to send commands to car
void sendCarCommand(const char *command)
{
  // command could be either "left", "right", "forward" or "reverse" or "stop"
  // or speed settingg "slow-speed", "normal-speed", or "fast-speed"
  if (strcmp(command, "up") == 0)
  {
    car.moveForward();
  }
  else if (strcmp(command, "down") == 0)
  {
    car.moveBackward();
  }
  else if (strcmp(command, "stop") == 0)
  {
    car.stop();
  }
  else if (strcmp(command, "slow-speed") == 0)
  {
    car.setCurrentSpeed(speedSettings::SLOW);
  }
  else if (strcmp(command, "normal-speed") == 0)
  {
    car.setCurrentSpeed(speedSettings::NORMAL);
  }
  else if (strcmp(command, "fast-speed") == 0)
  {
    car.setCurrentSpeed(speedSettings::FAST);
  } else if (strcmp(command, "auto") == 0) {
    switchControlMode(AUTO);
  } else if (strcmp(command, "manual") == 0) {
    switchControlMode(MANUAL);
  } else {
    int agleOfSteering = atoi(command);
    Serial.println(agleOfSteering);
    setSteeringAngle(agleOfSteering);
  }
}

// Function to send commands to servo
void sendServoCommand(const char *command)
{
  int angleOfSteering = atoi(command);
  Serial.println(angleOfSteering);
  setSteeringAngle(angleOfSteering);
}

// Callback function that receives messages from websocket client
void onCarWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type,
                  void *arg, uint8_t *data, size_t len)
{
  switch (type)
  {
  case WS_EVT_CONNECT:
    Serial.printf("carWs[%s][%u] connect\n", server->url(), client->id());
    client->printf("Hello Car Client %u :)", client->id());
    client->ping();
    break;

  case WS_EVT_DISCONNECT:
    Serial.printf("carWs[%s][%u] disconnect\n", server->url(), client->id());
    break;

  case WS_EVT_DATA:
  {
    AwsFrameInfo *info = (AwsFrameInfo *)arg;
    if (info->final && info->index == 0 && info->len == len)
    {
      if (info->opcode == WS_TEXT)
      {
        data[len] = 0;
        char *command = (char *)data;
        Serial.printf("carWs[%s][%u] command: %s\n", server->url(), client->id(), command);
        sendCarCommand(command);
      }
    }
    break;
  }

  case WS_EVT_PONG:
    Serial.printf("carWs[%s][%u] pong[%u]: %s\n", server->url(), client->id(), len, (len) ? (char *)data : "");
    break;

  case WS_EVT_ERROR:
    Serial.printf("carWs[%s][%u] error(%u): %s\n", server->url(), client->id(), *((uint16_t *)arg), (char *)data);
    break;
  }
}

// Callback function that receives messages from websocket client
void onServoWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type,
                    void *arg, uint8_t *data, size_t len)
{
  switch (type)
  {
  case WS_EVT_CONNECT:
    Serial.printf("servoWs[%s][%u] connect\n", server->url(), client->id());
    client->printf("Hello Servo Client %u :)", client->id());
    client->ping();
    break;

  case WS_EVT_DISCONNECT:
    Serial.printf("servoWs[%s][%u] disconnect\n", server->url(), client->id());
    break;

  case WS_EVT_DATA:
  {
    AwsFrameInfo *info = (AwsFrameInfo *)arg;
    if (info->final && info->index == 0 && info->len == len)
    {
      if (info->opcode == WS_TEXT)
      {
        data[len] = 0;
        char *command = (char *)data;
        Serial.printf("servoWs[%s][%u] command: %s\n", server->url(), client->id(), command);
        sendServoCommand(command);
      }
    }
    break;
  }

  case WS_EVT_PONG:
    Serial.printf("servoWs[%s][%u] pong[%u]: %s\n", server->url(), client->id(), len, (len) ? (char *)data : "");
    break;

  case WS_EVT_ERROR:
    Serial.printf("servoWs[%s][%u] error(%u): %s\n", server->url(), client->id(), *((uint16_t *)arg), (char *)data);
    break;
  }
}

// Processor for index.html page template.  This sets the radio button to checked or unchecked
String indexPageProcessor(const String &var)
{
  String status = "";
  if (var == "SPEED_SLOW_STATUS")
  {
    if (car.getCurrentSpeed() == speedSettings::SLOW)
    {
      status = "checked";
    }
  }
  else if (var == "SPEED_NORMAL_STATUS")
  {
    if (car.getCurrentSpeed() == speedSettings::NORMAL)
    {
      status = "checked";
    }
  }
  else if (var == "SPEED_FAST_STATUS")
  {
    if (car.getCurrentSpeed() == speedSettings::FAST)
    {
      status = "checked";
    }
  }
  return status;
}

// Function called when resource is not found on the server
void notFound(AsyncWebServerRequest *request)
{
  request->send(404, "text/plain", "Not found");
}

void listSPIFFSFiles()
{
  File root = SPIFFS.open("/");
  File file = root.openNextFile();
  while (file)
  {
    Serial.println(file.name()); // Print the file name
    file = root.openNextFile();
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

void setMotorSpeed(int speed, bool forward) {
  if (forward) {
      analogWrite(INA, speed);
      analogWrite(INB, 0);
  } else {
     analogWrite(INA, 0);
     analogWrite(INB, speed);
  }
}

void adjustServo(int angle) {
  angle = constrain(angle, MIN_ANGLE, MAX_ANGLE); // Giới hạn góc
  servo.write(angle);
}

void readDistanceTask(void *pvParameters) {
  
  while (true) {
        if (currentControlMode == MANUAL) {
        vTaskSuspend(NULL);  // Tạm dừng task hiện tại
    }

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

void navigationTask(void *Parameters) {
  while (true)
  {

    if (currentControlMode == MANUAL) {
      vTaskSuspend(NULL);  // Tạm dừng task hiện tại
    }
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
        setSteeringAngle(MIN_ANGLE); // Quay trái
    } else if (currentDirection == RIGHT) {
        setSteeringAngle(MAX_ANGLE); // Quay phải
    }
    
    setMotorSpeed(MOTOR_SPEED, false); // Lùi lại một chút
    delay(TURN_DELAY);
} else {
    // Trở về trạng thái cân bằng nếu không có vật cản
    setSteeringAngle(CENTER_ANGLE);
    setMotorSpeed(MOTOR_SPEED, true); // Tiếp tục tiến
    currentDirection = NONE; // Reset trạng thái
}

    vTaskDelay(pdMS_TO_TICKS(50));
  }
  
}

void switchControlMode(ControlMode mode) {
  currentControlMode = mode;
  if (mode == MANUAL) {
    vTaskSuspend(readDistanceTaskHandle);
    vTaskSuspend(navigationTaskHandle);
    car.stop();
    setSteeringAngle(CENTER_ANGLE);
  } else {
    vTaskResume(readDistanceTaskHandle);
    vTaskResume(navigationTaskHandle);
  }
}

// Setup function
void setup()
{
  // Initialize the serial monitor baud rate
  Serial.begin(115200);
  SPIFFS.begin();
  // other code
  server.serveStatic("/", SPIFFS, "/").setDefaultFile("/index.html");

  // Open the root directory
  File root = SPIFFS.open("/");
  File file = root.openNextFile();
  while (file)
  {
    Serial.println(file.name()); // Print the file name
    file = root.openNextFile();
  }

  // Attach the steering servo
  steeringServo.attach(SERVO_PIN);
  setSteeringAngle(90);

  Serial.println("Connecting to ");
  Serial.println(ssid);

  // Connect to your wifi
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  if (WiFi.waitForConnectResult() != WL_CONNECTED)
  {
    Serial.printf("WiFi Failed!\n");
    return;
  }

  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  // Initialize SPIFFS
  if (!SPIFFS.begin(true))
  {
    Serial.println("An Error has occurred while mounting SPIFFS");
    return;
  }

  // List files in SPIFFS
  listSPIFFSFiles();

  // Add callback function to websocket server
  carWs.onEvent(onCarWsEvent);
  servoWs.onEvent(onServoWsEvent);
  server.addHandler(&carWs);
  server.addHandler(&servoWs);

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
            {
              Serial.println("Requesting index page...");
              request->send(SPIFFS, "/index.html", "text/html", false, indexPageProcessor); });

  // Route to load entireframework.min.css file
  server.on("/test", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send(SPIFFS, "/test.html", "text/html"); });

  // Route to load entireframework.min.css file
  server.on("/css/entireframework.min.css", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send(SPIFFS, "/css/entireframework.min.css", "text/css"); });

  // Route to load custom.css file
  server.on("/css/custom.css", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send(SPIFFS, "/css/custom.css", "text/css"); });

  // Route to load custom.js file
  server.on("/js/custom.js", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send(SPIFFS, "/js/custom.js", "text/javascript"); });

  // On Not Found
  server.onNotFound(notFound);

  // Start server
  server.begin();

  distanceMutex = xSemaphoreCreateMutex();
  xTaskCreate(readDistanceTask, "ReadDistance", 2048, NULL, 1, &readDistanceTaskHandle);
  xTaskCreate(navigationTask, "Navigation", 2048, NULL, 1, &navigationTaskHandle);
}

void loop()
{
}
