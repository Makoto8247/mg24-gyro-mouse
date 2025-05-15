#include <Arduino.h>
#include <ArduinoLowPower.h>
#include <Wire.h>
#include <Adafruit_MPR121.h> // Capacitive touch sensor library
#include <LSM6DS3.h> // Gyro and accelerometer library
#include "MatrixButton.h"

#define READV_PIN D2         // 電圧測定ピン
#define POWER_BUTTON_PIN D0  // 電源ボタン
#define LED_PIN D1           // 電源確認LEDピン

#define LSM6DS3_ADDRESS 0x6A // I2C address for LSM6DS3
#define MPR121_ADDRESS 0x5A // I2C address for MPR121

LSM6DS3 gyroIMU(I2C_MODE, LSM6DS3_ADDRESS);
Adafruit_MPR121 touchSensor = Adafruit_MPR121();
MatrixButton matrixButton;

static uint16_t lastTouch = 0;
static uint16_t nextTouch = 0;

uint8_t readVoltage() {
  int sensorValue = analogRead(READV_PIN);
  float voltage = sensorValue * (3.3 / 4095.0); // Convert the ADC value to voltage (assuming 12-bit ADC and 3.3V reference)
  float voltagePercent = (voltage / 3.3) * 100; // Convert voltage to percentage
  
  Serial.print("Voltage: ");
  Serial.print(voltage, 2); // Print the voltage with 2 decimal places
  Serial.println(" V");

  return voltagePercent;
}

bool checkForDeepSleep() {
  if (digitalRead(POWER_BUTTON_PIN) == HIGH) {
    return false; // Button not pressed
  }

  bool setDeepSleep = false;
  unsigned long pressStartTime = millis();
  while (digitalRead(POWER_BUTTON_PIN) == LOW) {
    if (millis() - pressStartTime > 3000 && !setDeepSleep) { // 3 seconds of button press
      LowPower.attachInterruptWakeup(POWER_BUTTON_PIN, NULL, CHANGE);
      setDeepSleep = true;
    }
  }

  delay(10);
  return setDeepSleep;
}

void setup() {
  Serial.begin(115200);
  while(!Serial) delay(10);

  delay(50);

  // I2C setup to use speed 400kHz
  Wire.begin();
  delay(10);
  Wire.setClock(400000);
  
  pinMode(READV_PIN, INPUT);
  pinMode(POWER_BUTTON_PIN, INPUT);

  matrixButton.begin();
  
  while(gyroIMU.begin()) {
    Serial.println("Initializing LSM6DS3...");
    delay(500);
  }

  while (!touchSensor.begin(MPR121_ADDRESS)) {
    Serial.println("Initializing MPR121...");
    delay(500);
  }

  Serial.println("Setup complete.");
}

void loop() {
  //readVoltage();
  // マトリックスボタンをスキャン
  //matrixButton.scan();

  nextTouch = touchSensor.touched();
  Serial.print("Touch: ");
  Serial.println(nextTouch, HEX);

  Serial.print("Gyro AccelX: ");
  Serial.println(gyroIMU.readFloatAccelX());
  

  // ジャイロセンサーの値を読み取りマウスとして使用する
  //float accelX = gyroIMU.readFloatAccelX() * 10.0f;
  //float accelY = gyroIMU.readFloatAccelY() * -10.0f;
  
  // 加速度をマウス移動量に変換
  //int8_t mouseX = 0, mouseY = 0;
  
  // X軸の処理（しきい値処理）
  //if (accelX > 10) {
  //  mouseX = 10;
  //} else if (accelX < -10) {
  //  mouseX = -10;
  //} else {
  //  mouseX = (int8_t)accelX;
  //}
  
  //// Y軸の処理（しきい値処理）
  //if (accelY > 10) {
  //  mouseY = 10;
  //} else if (accelY < -10) {
  //  mouseY = -10;
  //} else {
  //  mouseY = (int8_t)accelY;
  //}
  

  delay(500);
  //lastTouch = nextTouch;

//  if(checkForDeepSleep()) {
//    LowPower.deepSleep();
//  }
}
