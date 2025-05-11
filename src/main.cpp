#include <Arduino.h>
#include <ArduinoLowPower.h>
#include <Adafruit_MPR121.h> // Capacitive touch library
#include <LSM6DS3.h> // Gyro and accelerometer library
#include "MatrixButton.h"

#define READV_PIN PA2         // 電圧測定ピン
#define POWER_BUTTON_PIN PD0  // 電源ボタン
#define LED_PIN D1            // 電源確認LEDピン

#define LSM6DS3_ADDRESS 0x6A // I2C address for LSM6DS3
#define MPR121_ADDRESS 0x5A // I2C address for MPR121
LSM6DS3 gyroIMU((unsigned char)I2C_MODE, (unsigned char)LSM6DS3_ADDRESS);
Adafruit_MPR121 touchSensor = Adafruit_MPR121();
MatrixButton matrixButton;

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
  while(!Serial);

  pinMode(READV_PIN, INPUT);
  pinMode(POWER_BUTTON_PIN, INPUT_PULLUP);

  matrixButton.begin();
  
  gyroIMU.begin();
  touchSensor.begin(MPR121_ADDRESS);

  Serial.println("Setup complete.");
}

void loop() {
  readVoltage();
    // マトリックスボタンをスキャン
  matrixButton.scan();
  
  Serial.print("Gyro AccelX: ");
  Serial.println(gyroIMU.readFloatAccelX());

  // ジャイロセンサーの値を読み取りマウスとして使用する
  float accelX = gyroIMU.readFloatAccelX() * 10.0f;
  float accelY = gyroIMU.readFloatAccelY() * -10.0f;
  
  // 加速度をマウス移動量に変換
  int8_t mouseX = 0, mouseY = 0;
  
  // X軸の処理（しきい値処理）
  if (accelX > 10) {
    mouseX = 10;
  } else if (accelX < -10) {
    mouseX = -10;
  } else {
    mouseX = (int8_t)accelX;
  }
  
  // Y軸の処理（しきい値処理）
  if (accelY > 10) {
    mouseY = 10;
  } else if (accelY < -10) {
    mouseY = -10;
  } else {
    mouseY = (int8_t)accelY;
  }
  

//  if(checkForDeepSleep()) {
//    LowPower.deepSleep();
//  }
}
