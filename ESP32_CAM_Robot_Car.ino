#include "esp_wifi.h"
#include "esp_camera.h"
#include <WiFi.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include <ESP32Servo.h>

// Access Point Credentials
const char* ssid1 = "ESP32-CAM Robot";
const char* password1 = "1234567890";

// Motor pins
#define RIGHT_M0 14
#define RIGHT_M1 15
#define LEFT_M0 13
#define LEFT_M1 12

// Ultrasonic pins
#define TRIG_PIN 0
#define ECHO_PIN 16

// Servo pin
#define SERVO_PIN 2

// Speed config
int speed = 80;

// Servo
Servo panServo;

// State machine
int state = 0;

// Start in manual mode; wait for web command to trigger autonomy
bool autonomous = false;
unsigned long stateStart = 0;

// RK4 Simulation
float dt = 0.05;
float servoTime = 0.0;
float estTime = 0.0;

// Camera config
#define CAMERA_MODEL_AI_THINKER
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

void startCameraServer();

void setMotors(int l0, int l1, int r0, int r1) {
  ledcWrite(5, l0);
  ledcWrite(6, l1);
  ledcWrite(3, r0);
  ledcWrite(4, r1);
}

void stopMotors() {
  setMotors(0, 0, 0, 0);
}

void moveForward() {
  setMotors(0, speed, 0, speed);
}

void moveLeftTurn() {
  setMotors(0, speed, speed, 0);
}

void moveRightTurn() {
  setMotors(speed, 0, 0, speed);
}

void moveBack() {
  setMotors(speed, 0, speed, 0);
}

void sweepServo() {
  for (int angle = 0; angle <= 135; angle += 45) {
    panServo.write(angle);
    delay(200);
    servoTime += 0.2;
  }
}

float rungeKutta(float t) {
  float h = 0.1, x = 0, y = 0;
  for (int i = 0; i < t / h; i++) {
    float k1 = x;
    float k2 = x + 0.5 * h * k1;
    float k3 = x + 0.5 * h * k2;
    float k4 = x + h * k3;
    y += (h / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4);
    x += h;
  }
  return y;
}

long readUltrasonic() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long duration = pulseIn(ECHO_PIN, HIGH);
  return duration * 0.034 / 2; // cm
}

void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
  Serial.begin(115200);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  ledcSetup(3, 2000, 8);
  ledcSetup(4, 2000, 8);
  ledcSetup(5, 2000, 8);
  ledcSetup(6, 2000, 8);

  ledcAttachPin(RIGHT_M0, 3);
  ledcAttachPin(RIGHT_M1, 4);
  ledcAttachPin(LEFT_M0, 5);
  ledcAttachPin(LEFT_M1, 6);

  panServo.attach(SERVO_PIN);
  panServo.write(90);

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

  if(psramFound()){
    config.frame_size = FRAMESIZE_QVGA;
    config.jpeg_quality = 10;
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_QVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x\n", err);
    return;
  }

  sensor_t * s = esp_camera_sensor_get();
  s->set_framesize(s, FRAMESIZE_QVGA);
  s->set_vflip(s, 1);
  s->set_hmirror(s, 1);

  WiFi.softAP(ssid1, password1);
  Serial.println(WiFi.softAPIP());
  startCameraServer();

  stateStart = millis();
}

void loop() {
  if (!autonomous) return;

  unsigned long now = millis();

  switch(state) {
    case 0:
      moveForward();
      if(now - stateStart > 2000) { stopMotors(); state++; stateStart = millis(); }
      break;
    case 1:
      sweepServo(); estTime = rungeKutta(servoTime); state++; stateStart = millis(); break;
    case 2:
      moveLeftTurn(); delay(250); stopMotors(); state++; stateStart = millis(); break;
    case 3:
      sweepServo(); estTime = rungeKutta(servoTime); state++; stateStart = millis(); break;
    case 4:
      moveRightTurn(); delay(500); stopMotors(); state++; stateStart = millis(); break;
    case 5:
      sweepServo(); estTime = rungeKutta(servoTime); state++; stateStart = millis(); break;
    case 6:
      moveLeftTurn(); delay(250); stopMotors(); state++; stateStart = millis(); break;
    case 7:
      moveForward();
      if(readUltrasonic() <= 100) {
        moveLeftTurn(); delay(250); stopMotors();
        moveForward(); delay(2000); stopMotors();
      }
      state++; stateStart = millis(); break;
    case 8:
      sweepServo(); estTime = rungeKutta(servoTime); state = 0; stateStart = millis(); break;
  }
}
