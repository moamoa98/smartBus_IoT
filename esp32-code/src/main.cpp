#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include "SPIFFS.h"
#include <ESP32Servo.h>

int currentDegree = 90;
int maxLeft = 50;
int maxRight = 140;

#define SERVO_PIN 13

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
  delay(200); // Đợi servo quay đến vị trí
}

class Car
{
private:
  // Motor pins
  int INA = 16;
  int INB = 17;

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
AsyncWebSocket ws("/ws");

// Our car object
Car car;

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
  }
  
  else
  {
    int agleOfSteering = atoi(command);
    Serial.println(agleOfSteering);
    setSteeringAngle(agleOfSteering);
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

// Callback function that receives messages from websocket client
void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type,
               void *arg, uint8_t *data, size_t len)
{
  switch (type)
  {
  case WS_EVT_CONNECT:
    Serial.printf("ws[%s][%u] connect\n", server->url(), client->id());
    client->printf("Hello Client %u :)", client->id());
    client->ping();
    break;

  case WS_EVT_DISCONNECT:
    Serial.printf("ws[%s][%u] disconnect\n", server->url(), client->id());
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
        Serial.printf("ws[%s][%u] command: %s\n", server->url(), client->id(), command);
        sendCarCommand(command);
      }
    }
    break;
  }

  case WS_EVT_PONG:
    Serial.printf("ws[%s][%u] pong[%u]: %s\n", server->url(), client->id(), len, (len) ? (char *)data : "");
    break;

  case WS_EVT_ERROR:
    Serial.printf("ws[%s][%u] error(%u): %s\n", server->url(), client->id(), *((uint16_t *)arg), (char *)data);
    break;
  }
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
  ws.onEvent(onWsEvent);
  server.addHandler(&ws);

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
}

void loop()
{
}
