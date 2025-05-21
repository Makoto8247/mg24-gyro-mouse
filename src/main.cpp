#include <Arduino.h>
#include <ArduinoLowPower.h>
#include <LSM6DS3.h> // ジャイロスコープとアクセロメータライブラリ
#include "MatrixButton.h"
#include "MPR121.h" // ソフトウェアI2C版MPR121を使用
#include "BLEConfig.h"
#include "MouseHID.h" // マウスHIDライブラリ

// ピン定義
#define READV_PIN D2         // 電圧測定ピン
#define POWER_BUTTON_PIN D0  // 電源ボタン
#define LED_PIN D1           // 電源確認LEDピン

#define LSM6DS3_ADDRESS 0x6A // LSM6DS3のI2Cアドレス
#define MPR121_ADDRESS 0x5A  // MPR121のI2Cアドレス

LSM6DS3 gyroIMU(I2C_MODE, LSM6DS3_ADDRESS);
//Adafruit_MPR121 touchSensor = Adafruit_MPR121(); // オリジナルライブラリをコメントアウト
MPR121 touchSensor; // ソフトウェアI2C版MPR121
MatrixButton matrixButton;

// HIDマウスオブジェクト
static mouse_data mouseData;
static mouse_accel_data mouseAccelData;

// HID レポート
static uint8_t reportArray[] = {0x00, 0x00, 0x00};
static uint8_t connectionHandle = SL_BT_INVALID_CONNECTION_HANDLE;
static uint32_t bondingHandle = SL_BT_INVALID_BONDING_HANDLE;
static uint16_t hidReport;

static uint16_t lastTouch = 0;
static uint16_t nextTouch = 0;
static unsigned long pressStartTime = 0;
static bool setDeepSleep = false;
static uint8_t batteryLevel = 0;

/***************************** BLE 実装 ************************************/
// BLE 設定
static bd_addr bleAddress = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
static uint8_t advertisingSetHandle = 0xFF;
static sl_status_t sc = SL_STATUS_OK;
static bool bleInit = false;

static void ble_initalize_gatt_db() {
  uint16_t gattdbSessionId;
  uint16_t service;
  uint16_t characteristic;
  uint16_t descriptor;

  sc = sl_bt_gattdb_new_session(&gattdbSessionId);
  app_assert_status(sc);

  // サービスの追加
  uint8_t genericAccessServiceUUID[] = {0x00, 0x18};
  sc = sl_bt_gattdb_add_service(
      gattdbSessionId,
      sl_bt_gattdb_primary_service,
      SL_BT_GATTDB_ADVERTISED_SERVICE,
      sizeof(genericAccessServiceUUID),
      genericAccessServiceUUID,
      &service);
  app_assert_status(sc);

  // キャラクタリスティックの追加
  sl_bt_uuid_16_t deviceNameUUID = { .data = {0x00, 0x2A}};
  sc = sl_bt_gattdb_add_uuid16_characteristic(
      gattdbSessionId,
      service,
      (SL_BT_GATTDB_CHARACTERISTIC_READ | SL_BT_GATTDB_CHARACTERISTIC_WRITE),
      0,
      0,
      deviceNameUUID,
      sl_bt_gattdb_fixed_length_value,
      strlen(BLE_NAME),
      strlen(BLE_NAME),
      (uint8_t *)BLE_NAME,
      &characteristic);
  app_assert_status(sc);

  // アピアランスの追加
  sl_bt_uuid_16_t appearanceUUID = { .data = {0x01, 0x2A}};
  const uint8_t appearanceValue[] = {0xC2, 0x03};
  sc = sl_bt_gattdb_add_uuid16_characteristic(
      gattdbSessionId,
      service,
      SL_BT_GATTDB_CHARACTERISTIC_READ,
      0,
      0,
      appearanceUUID,
      sl_bt_gattdb_fixed_length_value,
      sizeof(appearanceValue),
      sizeof(appearanceValue),
      appearanceValue,
      &characteristic);
  app_assert_status(sc);

  // サービスを開始
  sc = sl_bt_gattdb_start_service(gattdbSessionId, service);
  app_assert_status(sc);

  // バッテリーサービス
  const sl_bt_uuid_16_t batteryLevelUUID = { .data = {0x19, 0x2A}};
  const uint8_t BATTERY_LEVEL_INIT_VALUE = 100; // バッテリーの初期値
  sc = sl_bt_gattdb_add_uuid16_characteristic(
      gattdbSessionId,
      service,
      SL_BT_GATTDB_CHARACTERISTIC_READ,
      0,
      0,
      batteryLevelUUID,
      sl_bt_gattdb_fixed_length_value,
      sizeof(BATTERY_LEVEL_INIT_VALUE),
      sizeof(BATTERY_LEVEL_INIT_VALUE),
      &batteryLevel,
      &characteristic);
  app_assert_status(sc);

  // キャラクタデスクリプタの追加
  sl_bt_uuid_16_t charaPresentationFormatDescriptorUUID = { .data = {0x02, 0x29}};
  const uint8_t charaPresentationFormatValue[] = {0x01, 0x00, 0x00, 0x00, 0x00, 0x00};
  sc = sl_bt_gattdb_add_uuid16_descriptor(
      gattdbSessionId,
      characteristic,
      SL_BT_GATTDB_DESCRIPTOR_READ,
      0,
      charaPresentationFormatDescriptorUUID,
      sl_bt_gattdb_fixed_length_value,
      sizeof(charaPresentationFormatValue),
      sizeof(charaPresentationFormatValue),
      charaPresentationFormatValue,
      &descriptor);
  app_assert_status(sc);

  // クライアントデスクリプタの追加
  const sl_bt_uuid_16_t clientConfigurationDescriptorUUID = { .data = {0x02, 0x29}};
  const uint8_t clientConfigurationValue[] = {0x00, 0x00};
  sc = sl_bt_gattdb_add_uuid16_descriptor(
      gattdbSessionId,
      characteristic,
      SL_BT_GATTDB_DESCRIPTOR_READ | SL_BT_GATTDB_DESCRIPTOR_WRITE,
      0,
      clientConfigurationDescriptorUUID,
      sl_bt_gattdb_fixed_length_value,
      sizeof(clientConfigurationValue),
      sizeof(clientConfigurationValue),
      clientConfigurationValue,
      &descriptor);
  app_assert_status(sc);

  // バッテリーサービスの開始
  sc = sl_bt_gattdb_start_service(gattdbSessionId, service);
  app_assert_status(sc);

  // HIDサービスの追加
  sl_bt_uuid_16_t hid_protocol_mode_uuid = { .data = {0x4E, 0x2A}};
  const uint8_t HID_PROTOCOL_MODE_INIT_VALUE = 0x01;
  sc = sl_bt_gattdb_add_uuid16_characteristic(
      gattdbSessionId,
      service,
      (SL_BT_GATTDB_CHARACTERISTIC_READ | SL_BT_GATTDB_CHARACTERISTIC_WRITE_NO_RESPONSE),
      0,
      0,
      hid_protocol_mode_uuid,
      sl_bt_gattdb_fixed_length_value,
      sizeof(HID_PROTOCOL_MODE_INIT_VALUE),
      sizeof(HID_PROTOCOL_MODE_INIT_VALUE),
      &HID_PROTOCOL_MODE_INIT_VALUE,
      &characteristic);
  app_assert_status(sc);

  // HID レポート

}

static void ble_start_advertising() {
}
/**************************************************************************/


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
  
  pinMode(READV_PIN, INPUT);
  pinMode(POWER_BUTTON_PIN, INPUT);

  matrixButton.begin();
  while(gyroIMU.begin()) {
    Serial.println("Initializing LSM6DS3...");
    delay(500);
  }

  while (!touchSensor.begin(MPR121_ADDRESS, SDA, SCL)) {
    Serial.println("Initializing MPR121...");
    delay(500);
  }

  Serial.println("Setup complete.");
}

void loop() {
  batteryLevel = readVoltage();
  // マトリックスボタンをスキャン
  //matrixButton.scan();

  nextTouch = touchSensor.touched();
  if (nextTouch != lastTouch) {
    Serial.print("Touch: ");
    Serial.println(nextTouch, HEX);
    lastTouch = nextTouch;
  }
  delay(500);

  //Serial.print("Gyro AccelX: ");
  //Serial.println(gyroIMU.readFloatAccelX());
  

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
  

  //lastTouch = nextTouch;

//  if(checkForDeepSleep()) {
//    LowPower.deepSleep();
//  }
}
