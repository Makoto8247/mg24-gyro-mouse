# 自作マウス

# 気を付けるべきこと
## SDA, SCLピン番号を確認する
初期のライブラリだと
```cpp
// sl_i2cspm_xiao_mg24_1_config.h
// I2C0 SCL on PB2
#define SL_I2CSPM_XIAO_MG24_1_SCL_PORT                gpioPortB
#define SL_I2CSPM_XIAO_MG24_1_SCL_PIN                 3

// I2C0 SDA on PB3
#define SL_I2CSPM_XIAO_MG24_1_SDA_PORT                gpioPortB
#define SL_I2CSPM_XIAO_MG24_1_SDA_PIN                 2
```
となっているが、実際は
```cpp
// sl_i2cspm_xiao_mg24_1_config.h
// I2C0 SCL on PB2
#define SL_I2CSPM_XIAO_MG24_1_SCL_PORT                gpioPortB
#define SL_I2CSPM_XIAO_MG24_1_SCL_PIN                 2

// I2C0 SDA on PB3
#define SL_I2CSPM_XIAO_MG24_1_SDA_PORT                gpioPortB
#define SL_I2CSPM_XIAO_MG24_1_SDA_PIN                 3
```
と逆にしなければいけないので注意してください。

# エラー対処法
## flash driver 'efm32s2' not found
platformioで使用しているOpenOCDのバージョンが古い、または、種類の違うOpenOCDを使用している可能性があります。
使用している`openocd`のバージョンを確認してください。
```terminal
$ pio pkg list
```
このプロジェクトではtool-openocd@3.1200.0以上を推奨しています。
もしほかのバージョンがインストールされている場合(例`tool-openocd-esp32`など)は、以下のコマンドでアンインストールしてください。
```terminal
$ pio pkg uninstall <package_name>
```
