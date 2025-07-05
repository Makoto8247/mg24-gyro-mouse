#include <ArduinoLowPower.h>
#include <LSM6DS3.h> // ジャイロスコープとアクセロメータライブラリ
#include "MatrixButton.h" // マトリックスボタンライブラリ
#include "MPR121.h" // ソフトウェアI2C版MPR121を使用
#include "BLEConfig.h"
#include "MouseHID.h" // マウスHID

// ピン定義
#define READV_PIN D2         // 電圧測定ピン
#define POWER_BUTTON_PIN D0  // 電源ボタン
#define LED_PIN D1           // 電源確認LEDピン

#define LSM6DS3_ADDRESS 0x6A // LSM6DS3のI2Cアドレス
#define MPR121_ADDRESS 0x5A  // MPR121のI2Cアドレス

LSM6DS3 gyroIMU(I2C_MODE, LSM6DS3_ADDRESS);
MPR121 touchSensor; // ソフトウェアI2C版MPR121
MatrixButton matrixButton;

// HID レポート
static uint8_t reportArray[] = {0x00, 0x00, 0x00};
static uint8_t connectionHandle = SL_BT_INVALID_CONNECTION_HANDLE;
static uint32_t bondingHandle = SL_BT_INVALID_BONDING_HANDLE;
static uint16_t hidInputReport;

static uint16_t lastTouch = 0;
static uint16_t nextTouch = 0;
static unsigned long pressStartTime = 0;
static bool setDeepSleep = false;
static uint8_t batteryLevel = 0;

// HIDマウスオブジェクト
MouseHID mouseHID(reportArray);

// HID report map characteristic
static uint8_t HIDReportMapValue[] = { 0x05, 0x01, // Usage page (Generic Desktop)
                                      0x09, 0x02, // Usage (Mouse)
                                      0xA1, 0x01, // Collection (Application)
                                      0x09, 0x01, //   UsageId (Pointer)
                                      0xA1, 0x00, //   Collection (Physical)
                                      0x09, 0x30, //     UsageId (x)
                                      0x09, 0x31, //     UsageId (y)
                                      0x15, 0x80, //     LogicalMinimum(-128)
                                      0x25, 0x7F, //     LogicalMaximum(127)
                                      0x95, 0x02, //     ReportCount(2)
                                      0x75, 0x08, //     ReportSize(8)
                                      0x81, 0x06, //     Input(Data, Variable, Relative, NoWrap, Linear, PreferredState, NoNullPosition, BitField)
                                      0x05, 0x09, //     UsagePage(Button)
                                      0x19, 0x01, //     UsageIdMin(Button 1)
                                      0x29, 0x03, //     UsageIdMax(Button 3)
                                      0x15, 0x00, //     LogicalMinimum(0)
                                      0x25, 0x01, //     LogicalMaximum(1)
                                      0x95, 0x03, //     ReportCount(3)
                                      0x75, 0x01, //     ReportSize(1)
                                      0x81, 0x02, //     Input(Data, Variable, Absolute, NoWrap, Linear, PreferredState, NoNullPosition, BitField)
                                      0x95, 0x01, //     ReportCount(1)
                                      0x75, 0x05, //     ReportSize(5)
                                      0x81, 0x03, //     Input(Constant, Variable, Absolute, NoWrap, Linear, PreferredState, NoNullPosition, BitField)
                                      0xC0,       //   EndCollection()
                                      0xC0 };     // EndCollection()

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
  sl_bt_uuid_16_t charaPresentationFormatDescriptorUUID = { .data = {0x04, 0x29}};
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
  uint8_t hidServiceUUID[] = {0x12, 0x18};
  sc = sl_bt_gattdb_add_service(
      gattdbSessionId,
      sl_bt_gattdb_primary_service,
      SL_BT_GATTDB_ADVERTISED_SERVICE,
      sizeof(hidServiceUUID),
      hidServiceUUID,
      &service);
  app_assert_status(sc);

  // HID プロトコル
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

  // HID Input Report (実際のマウスデータ送信用)
  const sl_bt_uuid_16_t HIDInputReportUUID = { .data = {0x4D, 0x2A}};
  sc = sl_bt_gattdb_add_uuid16_characteristic(
      gattdbSessionId,
      service,
      (SL_BT_GATTDB_CHARACTERISTIC_READ | SL_BT_GATTDB_CHARACTERISTIC_NOTIFY),
      SL_BT_GATTDB_ENCRYPTED_READ,
      0,
      HIDInputReportUUID,
      sl_bt_gattdb_fixed_length_value,
      sizeof(reportArray),
      sizeof(reportArray),
      reportArray,
      &characteristic);
  app_assert_status(sc);
  hidInputReport = characteristic;

  // HID レポート
  const sl_bt_uuid_16_t HIDReportReferenceDescriptorUUID = { .data = {0x08, 0x2A}};
  const uint8_t HIDReportReferenceValue[] = { 0x00, 0x01};
  sc = sl_bt_gattdb_add_uuid16_descriptor(
      gattdbSessionId,
      characteristic,
      SL_BT_GATTDB_DESCRIPTOR_READ,
      SL_BT_GATTDB_ENCRYPTED_READ,
      HIDReportReferenceDescriptorUUID,
      sl_bt_gattdb_fixed_length_value,
      sizeof(HIDReportReferenceValue),
      sizeof(HIDReportReferenceValue),
      HIDReportReferenceValue,
      &descriptor);
  app_assert_status(sc);

  // HID レポートマップ
  const sl_bt_uuid_16_t HIDReportMapUUID = { .data = {0x4B, 0x2A}};
  sc = sl_bt_gattdb_add_uuid16_characteristic(
      gattdbSessionId,
      service,
      SL_BT_GATTDB_CHARACTERISTIC_READ,
      SL_BT_GATTDB_ENCRYPTED_READ,
      0,
      HIDReportMapUUID,
      sl_bt_gattdb_fixed_length_value,
      sizeof(HIDReportMapValue),
      sizeof(HIDReportMapValue),
      HIDReportMapValue,
      &characteristic);
  app_assert_status(sc);

  // HID レポートマップのデスクリプタ
  const sl_bt_uuid_16_t HIDReportMapDescriptorUUID = { .data = {0x4B, 0x2A}};
  const uint8_t HIDExternalReportReferenceValue[] = {0x00, 0x00};
  sc = sl_bt_gattdb_add_uuid16_descriptor(
      gattdbSessionId,
      characteristic,
      SL_BT_GATTDB_DESCRIPTOR_READ,
      0,
      HIDReportMapDescriptorUUID,
      sl_bt_gattdb_fixed_length_value,
      sizeof(HIDExternalReportReferenceValue),
      sizeof(HIDExternalReportReferenceValue),
      HIDExternalReportReferenceValue,
      &descriptor);
  app_assert_status(sc);

  // HID 情報の追加
  const sl_bt_uuid_16_t HIDInformationUUID = { .data = {0x4A, 0x2A}};
  const uint8_t HIDInformationValue[] = {0x00, 0x11, 0x00, 0x02};
  sc = sl_bt_gattdb_add_uuid16_characteristic(
      gattdbSessionId,
      service,
      SL_BT_GATTDB_CHARACTERISTIC_READ,
      0,
      0,
      HIDInformationUUID,
      sl_bt_gattdb_fixed_length_value,
      sizeof(HIDInformationValue),
      sizeof(HIDInformationValue),
      HIDInformationValue,
      &characteristic);
  app_assert_status(sc);

  // HID コントロールポイント
  const sl_bt_uuid_16_t HIDControlPointUUID = { .data = {0x4C, 0x2A}};
  const uint8_t HIDControlPointValue[] = {0x00};
  sc = sl_bt_gattdb_add_uuid16_characteristic(
      gattdbSessionId,
      service,
      SL_BT_GATTDB_CHARACTERISTIC_WRITE_NO_RESPONSE,
      0,
      0,
      HIDControlPointUUID,
      sl_bt_gattdb_fixed_length_value,
      sizeof(HIDControlPointValue),
      sizeof(HIDControlPointValue),
      HIDControlPointValue,
      &characteristic);
  app_assert_status(sc);

  // HID サービスの開始
  sc = sl_bt_gattdb_start_service(gattdbSessionId, service);
  app_assert_status(sc);

  // GATT DB コミット
  sc = sl_bt_gattdb_commit(gattdbSessionId);
  app_assert_status(sc);
}

static void ble_start_advertising() {
  static uint8_t advertisingSetHandle = 0xFF;
  bleInit = true;

  if (bleInit) {
    sc = sl_bt_advertiser_create_set(&advertisingSetHandle);
    app_assert_status(sc);

    sc = sl_bt_advertiser_set_timing(
        advertisingSetHandle,
        160,
        160,
        0,
        0);
    app_assert_status(sc);
    bleInit = false;
  }

  sc = sl_bt_legacy_advertiser_generate_data(advertisingSetHandle, sl_bt_advertiser_general_discoverable);
  app_assert_status(sc);

  sc = sl_bt_legacy_advertiser_start(advertisingSetHandle, sl_bt_advertiser_connectable_scannable);
  app_assert_status(sc);

  Serial.println("Advertising started.");
  Serial.print("Advertising set handle: ");
  Serial.println(BLE_NAME);
}
sl_bt_msg_t evt;
void sl_bt_on_event(sl_bt_msg_t *evt) {
  uint8_t bleAddressType;
  
  switch (SL_BT_MSG_ID(evt->header)) {
    case sl_bt_evt_system_boot_id:
      Serial.println("BLE System Boot");

      sc = sl_bt_system_get_identity_address(&bleAddress, &bleAddressType);
      app_assert_status(sc);

      // GATT DBの初期化
      ble_initalize_gatt_db();
      
      // HID input devices requires mandatory secure level and bonding
      sc = sl_bt_sm_configure(0, sl_bt_sm_io_capability_noinputnooutput);
      app_assert_status(sc);

      // Allow bonding
      sc = sl_bt_sm_set_bondable_mode(1);
      app_assert_status(sc);
      
      // アドバタイジング開始
      ble_start_advertising();
      break;

    case sl_bt_evt_connection_opened_id:
      connectionHandle = evt->data.evt_connection_opened.connection;
      bondingHandle = evt->data.evt_connection_opened.bonding;
      Serial.print("Connection opened: ");
      Serial.println(connectionHandle);
      
      if (bondingHandle == SL_BT_INVALID_BONDING_HANDLE) {
        Serial.println("Connection not bonded yet");
      } else {
        Serial.println("Connection bonded");
      }

      Serial.println("Increase security");
      sc = sl_bt_sm_increase_security(evt->data.evt_connection_opened.connection);
      app_assert_status(sc);
      break;

    case sl_bt_evt_connection_closed_id:
      connectionHandle = SL_BT_INVALID_CONNECTION_HANDLE;
      bondingHandle = SL_BT_INVALID_BONDING_HANDLE;
      Serial.println("Connection closed, restarting advertising");
      
      // 再アドバタイジング
      ble_start_advertising();
      break;

    case sl_bt_evt_sm_bonded_id:
      bondingHandle = evt->data.evt_sm_bonded.bonding;
      connectionHandle = evt->data.evt_sm_bonded.connection;
      Serial.print("Bonded - handle: 0x");
      Serial.print(evt->data.evt_sm_bonded.connection, HEX);
      Serial.print(" - security mode: 0x");
      Serial.println(evt->data.evt_sm_bonded.security_mode, HEX);
      break;

    case sl_bt_evt_sm_bonding_failed_id:
      Serial.println("Bonding failed");
      break;

    case sl_bt_evt_gatt_server_characteristic_status_id:
      // クライアントがnotificationを有効/無効にした時
      if (evt->data.evt_gatt_server_characteristic_status.characteristic == hidInputReport) {
        if (evt->data.evt_gatt_server_characteristic_status.status_flags == sl_bt_gatt_server_client_config) {
          if (evt->data.evt_gatt_server_characteristic_status.client_config_flags == sl_bt_gatt_server_notification) {
            Serial.println("HID Input Report notifications enabled");
          } else {
            Serial.println("HID Input Report notifications disabled");
          }
        }
      }
      break;

    default:
      break;
  }
}
/**************************************************************************/


uint8_t readVoltage() {
  int sensorValue = analogRead(READV_PIN);
  float voltage = sensorValue * (3.3 / 4095.0); // Convert the ADC value to voltage (assuming 12-bit ADC and 3.3V reference)
  float voltagePercent = (voltage / 3.3) * 100; // Convert voltage to percentage

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

  // 加速度センサーから値を読み取る（元のサンプルと同じ方式）
  // 軸の向きを元のサンプルに合わせて調整
  int32_t acc_y = (int32_t)(gyroIMU.readFloatAccelX() * 10.0f);      // X軸の値をY軸に
  int32_t acc_x = (int32_t)(gyroIMU.readFloatAccelY() * 10.0f * -1.0f); // Y軸の値を反転してX軸に

  // 元のサンプルと同じ閾値処理
  #define IMU_ACC_X_THRESHOLD 10
  #define IMU_ACC_Y_THRESHOLD 10
  
  float mouseX, mouseY;
  
  if (acc_x > IMU_ACC_X_THRESHOLD) {
    mouseX = IMU_ACC_X_THRESHOLD;
  } else if (acc_x < (-1 * IMU_ACC_X_THRESHOLD)) {
    mouseX = (-1 * IMU_ACC_X_THRESHOLD);
  } else {
    mouseX = acc_x;
  }

  if (acc_y > IMU_ACC_Y_THRESHOLD) {
    mouseY = IMU_ACC_Y_THRESHOLD;
  } else if (acc_y < (-1 * IMU_ACC_Y_THRESHOLD)) {
    mouseY = (-1 * IMU_ACC_Y_THRESHOLD);
  } else {
    mouseY = acc_y;
  }

  mouseHID.processAcceleration(mouseX, mouseY);

  // デバッグ情報を追加
  Serial.print("Connection: ");
  Serial.print(connectionHandle != SL_BT_INVALID_CONNECTION_HANDLE ? "OK" : "NO");
  Serial.print(", Bonding: ");
  Serial.print(bondingHandle != SL_BT_INVALID_BONDING_HANDLE ? "OK" : "NO");
  Serial.print(", acc_x: ");
  Serial.print(acc_x);
  Serial.print(", acc_y: ");
  Serial.print(acc_y);
  Serial.print(", mouseX: ");
  Serial.print(mouseX);
  Serial.print(", mouseY: ");
  Serial.print(mouseY);
  Serial.print(", reportArray: [");
  Serial.print((int8_t)reportArray[0]);
  Serial.print(", ");
  Serial.print((int8_t)reportArray[1]);
  Serial.print(", ");
  Serial.print(reportArray[2]);
  Serial.println("]");

  // BLE送信（元のサンプルと同じ条件：接続とボンディングが両方完了している場合のみ）
  if (connectionHandle != SL_BT_INVALID_CONNECTION_HANDLE && bondingHandle != SL_BT_INVALID_BONDING_HANDLE) {
    // データに変化があるか、または移動量が0でない場合のみ送信
    if (reportArray[0] != 0 || reportArray[1] != 0 || reportArray[2] != 0) {
      Serial.print("cursor [delta-X: ");
      Serial.print((int8_t)reportArray[0], DEC);
      Serial.print(" delta-Y: ");
      Serial.print((int8_t)reportArray[1], DEC);
      Serial.print(" ] LMB: ");
      Serial.println(reportArray[2], HEX);
      
      sc = sl_bt_gatt_server_notify_all(hidInputReport, sizeof(reportArray), reportArray);
      if (sc != SL_STATUS_OK) {
        Serial.print("sl_bt_gatt_server_notify_all() returned with error code 0x");
        Serial.println(sc, HEX);
      }
    } else {
      Serial.println("No movement data to send");
    }
  } else {
    Serial.println("Not ready to send (connection or bonding missing)");
  }

  // 少し遅延を入れてBLEスタックの負荷を軽減
  delay(10);

//  if(checkForDeepSleep()) {
//    LowPower.deepSleep();
//  }
}


#ifndef BLE_STACK_SILABS
  #error "This example is only compatible with the Silicon Labs BLE stack. Please select 'BLE (Silabs)' in 'Tools > Protocol stack'."
#endif
