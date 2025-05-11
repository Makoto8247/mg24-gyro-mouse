# 自作マウス


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
