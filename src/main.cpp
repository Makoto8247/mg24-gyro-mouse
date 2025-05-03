#include <Arduino.h>
#include <Wire.h>
#include <LSM6DS3.h>
#include "ArduinoLowPower.h"

#define READV_PIN PA3
#define POWER_PIN PD1 // Power button pin
#define MT_OUT_PIN1 1 // Matrix pin
#define MT_OUT_PIN2 2 // Matrix pin 2
#define MT_OUT_PIN3 3 // Matrix pin 3
#define MT_IN_PIN1 4 // Matrix pin 4
#define MT_IN_PIN2 5 // Matrix pin 5
#define MT_IN_PIN3 6 // Matrix pin 6

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

// Create a instance of class LSM6DS3
LSM6DS3 myIMU(SPI_MODE, 0x6A);

void setup() {
  Serial.begin(115200);
  while(!Serial);

  pinMode(READV_PIN, INPUT);
  pinMode(POWER_PIN, INPUT_PULLUP);

  initializeMatrixPins();
  
}

void readVoltage() {
  int sensorValue = analogRead(READV_PIN);
  float voltage = sensorValue * (3.3 / 4095.0); // Convert the ADC value to voltage (assuming 12-bit ADC and 3.3V reference)
  
  Serial.print("Voltage: ");
  Serial.print(voltage, 2); // Print the voltage with 2 decimal places
  Serial.println(" V");
}

void startDeepSleep() {
  if (digitalRead(POWER_PIN) == LOW) {
    LowPower.attachInterruptWakeup(POWER_PIN, NULL, CHANGE);
  }
}

void loop() {
  readVoltage();
}
