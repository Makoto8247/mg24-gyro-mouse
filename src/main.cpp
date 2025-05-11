#include <Arduino.h>
#include <ArduinoLowPower.h>
#include <Wire.h>
#include <Adafruit_MPR121.h> // Capacitive touch library
#include <LSM6DS3.h> // Gyro and accelerometer library

#define READV_PIN PA2
#define POWER_BUTTON_PIN PD0 // Power button pin
#define LED_PIN D1
#define SPI_CS_PIN PD2 // Chip select pin for SPI


// BLE
#define BLE_NAME "BLE_MG24_MOUSE"

// Mouse
struct mouseData {
  int8_t delta_x;
  int8_t delta_y;
  uint8_t buttons;
};
static mouseData report_mouse;
static uint8_t report_mouse_array[3];

static uint8_t connection_handle = SL_BT_INVALID_CONNECTION_HANDLE;
static uint32_t bonding_handle = SL_BT_INVALID_BONDING_HANDLE;
static uint16_t hid_report;

static void ble_initialize_gatt_db() {
  sl_status_t status;
  uint16_t gattdb_session_id;
  uint16_t service;
  uint16_t characteristic;
  uint16_t descriptor;

  // Create a new GATT database session
  status = sl_bt_gattdb_new_session(&gattdb_session_id);
  app_assert_status(status);

  // Device Information Service
  sl_bt_uuid_16_t device_name_uuid = { .data = {0x2A, 0x00} };
  status = sl_bt_gattdb_add_uuid16_characteristic(gattdb_session_id,
                                                  service,
                                                  (SL_BT_GATTDB_CHARACTERISTIC_READ | SL_BT_GATTDB_CHARACTERISTIC_WRITE),
                                                  0,
                                                  0,
                                                  device_name_uuid,
                                                  sl_bt_gattdb_fixed_length_value,
                                                  strlen(BLE_NAME),
                                                  strlen(BLE_NAME),
                                                  (uint8_t *)BLE_NAME,
                                                  &characteristic);
  app_assert_status(status);
}

/*
 * ボタンマトリックス割り当て表
 * 
 * OUTx + INy の組み合わせで対応する入力を定義
 * 
 * ┌────────┬─────────────────┬─────────────────┬───────────────┐
 * │        │    IN1          │    IN2          │    IN3        │
 * ├────────┼─────────────────┼─────────────────┼───────────────┤
 * │ OUT1   │ マウス左クリック │ キーボードJキー  │ マウス戻るボタン│
 * │ OUT2   │ キーボードIキー  │ ENTERキー       │ キーボードKキー │
 * │ OUT3   │ マウス右クリック │ キーボードLキー  │ マウス進むボタン│
 * └────────┴─────────────────┴────────────────┴────────────────┘
 */
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

#define LSM6DS3_ADDRESS 0x6A // I2C address for LSM6DS3
LSM6DS3 gyroIMU((unsigned char)I2C_MODE, (unsigned char)LSM6DS3_ADDRESS);
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
  Serial.print("Gyro AccelX: ");
  Serial.println(gyroIMU.readFloatAccelX());
  Serial.print("Gyro AccelY: ");
  Serial.println(gyroIMU.readFloatAccelY());
  Serial.print("Gyro AccelZ: ");
  Serial.println(gyroIMU.readFloatAccelZ());
  Serial.print("Gyro GyroX: ");
  Serial.println(gyroIMU.readFloatGyroX());
  Serial.print("Gyro GyroY: ");
  Serial.println(gyroIMU.readFloatGyroY());
  Serial.print("Gyro GyroZ: ");
  Serial.println(gyroIMU.readFloatGyroZ());

//  if(checkForDeepSleep()) {
//    LowPower.deepSleep();
//  }
}
