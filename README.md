# robocon2022_essentials

## 概要
Robot Contest 2022 Aチーム ROSパッケージ  
Object detection & motion capture,UART,Movement,Controllerを統括します

## 環境

### 機械学習、ROS Master用PC
- RayTrek gaming pc
- Ubuntu 20.04
- ROS Noetic

### ロボット用PC
- ThinkPad
- Ubuntu 20.04
- ROS Noetic

### コントローラー用PC
- Raspberry pi 4 model B
- Rasbian (debian)
- ROS Noetic
- Dual shock 4

## 各種パッケージ

### [robocon2022_essentials](/robocon2022_essentials/)
Meta package

### [robocon2022_essentials_serial](/robocon2022_essentials_serial/)
PC <-> STM32間の通信を担当  
通信ライブラリとして、独自の通信ライブラリ(SerialBridge)を採用しています

### [robocon2022_essentials_msgs](/robocon2022_essentials_msgs/)
Topic Message定義用パッケージ

### [robocon2022_essentials_detection](/robocon2022_essentials_detection/)
[nuitrack_body_tracker](https://github.com/TNCT-Mechatech/nuitrack_body_tracker)からのメッセージのフォーマット&フィルター

### [robocon2022_essentials_visualizer](/robocon2022_essentials_visualizer/)
各種メッセージの可視化

### [robocon2022_essentials_controller](/robocon2022_essentials_controller/)
メインコントローラー

### [robocon2022_essentials_control](/robocon2022_essentials_control/)
Launchファイル等

## License
MIT LICENSE  
Copyright © TNCT-Mechatech
