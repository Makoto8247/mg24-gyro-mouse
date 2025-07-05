#include "MPR121.h"

MPR121::MPR121() {
  softWire = NULL;
  _i2caddr = 0;
}

MPR121::~MPR121() {
  if (softWire) {
    delete softWire;
    softWire = NULL;
  }
}

bool MPR121::begin(uint8_t i2caddr, uint8_t sda_pin, uint8_t scl_pin,
                   uint8_t touch_threshold, uint8_t release_threshold) {

    if (softWire) {
        delete softWire;
    }
    softWire = new SoftWire(sda_pin, scl_pin);

    // ソフトウェアI2C用のバッファを割り当て
    Serial.println("Allocating buffers for SoftWire");
    static uint8_t rxBuffer[32];
    static uint8_t txBuffer[32];
    softWire->setRxBuffer(rxBuffer, sizeof(rxBuffer));
    softWire->setTxBuffer(txBuffer, sizeof(txBuffer));

    Serial.println("Initializing SoftWire");
    softWire->begin();
    // クロックを低速に設定（安定性のため）
    softWire->setClock(100000);
    
    // 通信テスト
    Serial.print("Testing I2C address: 0x");
    Serial.println(i2caddr, HEX);
    softWire->beginTransmission(i2caddr);
    uint8_t err = softWire->endTransmission();
    if (err != 0) {
        return false;
    }
    
    _i2caddr = i2caddr;

    // ソフトリセット
    Serial.println("Soft reset");
    writeRegister(MPR121_SOFTRESET, 0x63);
    delay(10); // 1msから10msに延長
    
    // 動作設定レジスタをゼロに（スタンバイモード）
    Serial.println("Setting ECR to 0");
    writeRegister(MPR121_ECR, 0x0);

    // CONFIG2レジスタで動作確認
    Serial.println("Checking CONFIG2");
    uint8_t c = readRegister8(MPR121_CONFIG2);
    if (c != 0x24) {
        return false;
    }

    // タッチと離れた時のしきい値を設定
    Serial.println("Setting thresholds");
    setThresholds(touch_threshold, release_threshold);
    
    // デジタルフィルタ設定
    Serial.println("Setting digital filter");
    writeRegister(MPR121_MHDR, 0x01);
    writeRegister(MPR121_NHDR, 0x01);
    writeRegister(MPR121_NCLR, 0x0E);
    writeRegister(MPR121_FDLR, 0x00);

    writeRegister(MPR121_MHDF, 0x01);
    writeRegister(MPR121_NHDF, 0x05);
    writeRegister(MPR121_NCLF, 0x01);
    writeRegister(MPR121_FDLF, 0x00);

    writeRegister(MPR121_NHDT, 0x00);
    writeRegister(MPR121_NCLT, 0x00);
    writeRegister(MPR121_FDLT, 0x00);

    // その他の設定
    writeRegister(MPR121_DEBOUNCE, 0);
    writeRegister(MPR121_CONFIG1, 0x10); // 16uA充電電流
    writeRegister(MPR121_CONFIG2, 0x20); // 0.5uSエンコード、1ms周期

    // 電極を有効化してMPR121を起動
    Serial.println("Setting ECR");
    byte ECR_SETTING = B10000000 + 12; // ベースライントラッキング用5ビット + 近接検知無効 + 電極12個
    writeRegister(MPR121_ECR, ECR_SETTING); 

    return true;
}

// setThresholds関数の実装
void MPR121::setThresholds(uint8_t touch, uint8_t release) {
  // すべての電極に同じしきい値を設定
  for (uint8_t i = 0; i < 12; i++) {
    writeRegister(MPR121_TOUCHTH_0 + 2 * i, touch);
    writeRegister(MPR121_RELEASETH_0 + 2 * i, release);
  }
}

// タッチ状態の取得
uint16_t MPR121::touched() {
  uint16_t t = readRegister16(MPR121_TOUCHSTATUS_L);
  return t & 0x0FFF;
}

// フィルタ処理されたデータの取得
uint16_t MPR121::filteredData(uint8_t t) {
  if (t > 12)
    return 0;
  return readRegister16(MPR121_FILTDATA_0L + t * 2);
}

// ベースラインデータの取得
uint16_t MPR121::baselineData(uint8_t t) {
  if (t > 12)
    return 0;
  uint16_t bl = readRegister8(MPR121_BASELINE_0 + t);
  return (bl << 2);
}

// 8ビットレジスタの読み取り
uint8_t MPR121::readRegister8(uint8_t reg) {
  softWire->beginTransmission(_i2caddr);
  softWire->write(reg);
  softWire->endTransmission(false); // リピートスタート

  uint8_t value = 0;
  if (softWire->requestFrom(_i2caddr, (uint8_t)1) == 1) {
    value = softWire->read();
  }
  
  return value;
}

// 16ビットレジスタの読み取り (LSBファースト)
uint16_t MPR121::readRegister16(uint8_t reg) {
  uint16_t value = 0;
  
  softWire->beginTransmission(_i2caddr);
  softWire->write(reg);
  softWire->endTransmission(false); // リピートスタート
  
  if (softWire->requestFrom(_i2caddr, (uint8_t)2) == 2) {
    value = softWire->read();       // 下位バイト
    value |= ((uint16_t)softWire->read() << 8); // 上位バイト
  }
  
  return value;
}

// レジスタへの書き込み
void MPR121::writeRegister(uint8_t reg, uint8_t value) {
  // MPR121はほとんどのレジスタに書き込むためにはスタンバイモードにする必要がある
  bool stop_required = true;
  uint8_t ecr_backup = 0;
  
  // ECRレジスタの現在の値を取得
  if ((reg == MPR121_ECR) || ((0x73 <= reg) && (reg <= 0x7A))) {
    stop_required = false;
  } else {
    ecr_backup = readRegister8(MPR121_ECR);
    // スタンバイモードに設定
    softWire->beginTransmission(_i2caddr);
    softWire->write(MPR121_ECR);
    softWire->write(0x00);
    softWire->endTransmission();
  }
  
  // レジスタに値を書き込む
  softWire->beginTransmission(_i2caddr);
  softWire->write(reg);
  softWire->write(value);
  softWire->endTransmission();
  
  if (stop_required) {
    // ECRの以前の設定を復元
    softWire->beginTransmission(_i2caddr);
    softWire->write(MPR121_ECR);
    softWire->write(ecr_backup);
    softWire->endTransmission();
  }
}