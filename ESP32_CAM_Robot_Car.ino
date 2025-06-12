#include "esp_camera.h"
#include <WiFi.h>
#include "soc/rtc_cntl_reg.h"

// WiFi
const char* ssid = "HUAWEI-2.4G-6b7j";
const char* password = "7aakc7ce";
//final
// AI Thinker Camera Pin Mapping
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

// Motor Pins
#define RIGHT_M0 14
#define RIGHT_M1 15
#define LEFT_M0 13
#define LEFT_M1 12

// PWM Channels
#define CH_RIGHT_M0 3
#define CH_RIGHT_M1 4
#define CH_LEFT_M0 5
#define CH_LEFT_M1 6

int speed = 70;  // slow speed
bool autonomous = true;
int state = 0;
unsigned long stateStart = 0;

void startCameraServer();

void setMotors(int l0, int l1, int r0, int r1) {
  ledcWrite(CH_LEFT_M0, l0);
  ledcWrite(CH_LEFT_M1, l1);
  ledcWrite(CH_RIGHT_M0, r0);
  ledcWrite(CH_RIGHT_M1, r1);
}

void stopMotors() {
  setMotors(0, 0, 0, 0);
}

void moveForward() {
  setMotors(0, speed, 0, speed);
}

void turnLeft() {
  setMotors(0, speed, speed, 0);
}

void turnRight() {
  setMotors(speed, 0, 0, speed);
}

void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
  Serial.begin(115200);

  // Motor PWM setup
  ledcSetup(CH_RIGHT_M0, 1000, 8); ledcAttachPin(RIGHT_M0, CH_RIGHT_M0);
  ledcSetup(CH_RIGHT_M1, 1000, 8); ledcAttachPin(RIGHT_M1, CH_RIGHT_M1);
  ledcSetup(CH_LEFT_M0, 1000, 8);  ledcAttachPin(LEFT_M0, CH_LEFT_M0);
  ledcSetup(CH_LEFT_M1, 1000, 8);  ledcAttachPin(LEFT_M1, CH_LEFT_M1);

  // Camera config
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
    config.frame_size = FRAMESIZE_QVGA;
    config.jpeg_quality = 10;
    config.fb_count = 1; // safer for freeze issues
  } else {
    config.frame_size = FRAMESIZE_QVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }

  if (esp_camera_init(&config) != ESP_OK) {
    Serial.println("Camera init failed");
    return;
  }

  WiFi.begin(ssid, password);
  Serial.print("Connecting");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500); Serial.print(".");
  }
  Serial.println("\nWiFi connected");
  Serial.print("Stream: http://"); Serial.print(WiFi.localIP()); Serial.println(":81/stream");

  startCameraServer();
  delay(3000);
  stateStart = millis();
}

void loop() {
  if (!autonomous) return;

  unsigned long now = millis();

  switch (state) {
    case 0:
      moveForward();
      if (now - stateStart > 1500) {
        stopMotors();
        state++; stateStart = millis();
      }
      break;

    case 1:
      if (now - stateStart > 3000) {  // wait 3 secs after moveForward
        turnLeft();
        state++; stateStart = millis();
      }
      break;

    case 2:
      if (now - stateStart > 400) {
        stopMotors();
        state++; stateStart = millis();
      }
      break;

    case 3:
      moveForward();
      if (now - stateStart > 1500) {
        stopMotors();
        state++; stateStart = millis();
      }
      break;

    case 4:
      if (now - stateStart > 3000) {
        turnRight();
        state++; stateStart = millis();
      }
      break;

    case 5:
      if (now - stateStart > 500) {
        stopMotors();
        state++; stateStart = millis();
      }
      break;

    case 6:
      moveForward();
      if (now - stateStart > 1500) {
        stopMotors();
        state++; stateStart = millis();
      }
      break;

    case 7:
      if (now - stateStart > 3000) {
        state = 0;  // restart cycle
        stateStart = millis();
      }
      break;
  }
}
