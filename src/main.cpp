#include <Arduino.h>
#include <Wire.h>
#include <ICM20689.h> // Gyro and accelerometer library
#include <Adafruit_MPR121.h> // Capacitive touch library
#include "ArduinoLowPower.h"

#define READV_PIN PA2
#define POWER_BUTTON_PIN PD0 // Power button pin
#define LED_PIN D1
#define SPI_CS_PIN PD2 // Chip select pin for SPI
#define MT_OUT_PIN1 D10 // Matrix pin
#define MT_OUT_PIN2 D9 // Matrix pin 2
#define MT_OUT_PIN3 D8 // Matrix pin 3
#define MT_IN_PIN1 D7 // Matrix pin 4
#define MT_IN_PIN2 D6 // Matrix pin 5
#define MT_IN_PIN3 D3 // Matrix pin 6

int mt_out_pins[] = {MT_OUT_PIN1, MT_OUT_PIN2, MT_OUT_PIN3};
int mt_in_pins[] = {MT_IN_PIN1, MT_IN_PIN2, MT_IN_PIN3};

void initializeMatrixPins() {
  for (int i = 0; i < 3; i++) {
    pinMode(mt_out_pins[i], OUTPUT);
  }
  for (int i = 0; i < 3; i++) {
    pinMode(mt_in_pins[i], INPUT_PULLUP);
  }
}

ICM20689 gyroIMU(SPI, SPI_CS_PIN);
Adafruit_MPR121 touchSensor = Adafruit_MPR121();

void readVoltage() {
  int sensorValue = analogRead(READV_PIN);
  float voltage = sensorValue * (3.3 / 4095.0); // Convert the ADC value to voltage (assuming 12-bit ADC and 3.3V reference)
  
  Serial.print("Voltage: ");
  Serial.print(voltage, 2); // Print the voltage with 2 decimal places
  Serial.println(" V");
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

  initializeMatrixPins();

  gyroIMU.begin();
  touchSensor.begin(0x5A);
  
}

void loop() {
  readVoltage();
  Serial.println(gyroIMU.readSensor());

//  if(checkForDeepSleep()) {
//    LowPower.deepSleep();
//  }
}
