# Dummy1 MCU

## 소개
[Dummy1](https://github.com/Gone030/Dummy1)의 MCU(Arduino Due)의 코드 입니다. Encoder, IMU, DC Motor, Servo motor를 제어합니다.
추가적으로 PID 제어를 위한 튜닝을 하는 코드가 별도의 폴더로 추가되어 있습니다.

## 부품 소개
* Robot Computer : Jetson Nano
* MCU : Arduino DUE
* Encoder : E30S4-3000-3-V-5 Rotary Encoder
* IMU : MPU9250
* Motor Driver : BTS7960
* DC Motor : JGA25-370 소형DC기어드 모터 (176 RPM)
* Servo Motor : MG996R
* Lidar : YDLidar X4
* Battery : 7.2V 5000mAh NiMH Battery

## Robot 소개

![Robot](https://user-images.githubusercontent.com/89852937/233829878-d56b3ca0-411f-435c-833b-91b7ba53c3c0.png)

Dummy1은 Ackermann Steering을 사용해 회전하는 Car-like Robot입니다. 개략적인 Robot의 모습은 위 그림과 같습니다.
로봇의 뒤쪽에 배치된 DC모터로 후륜구동하며 앞쪽에 배치된 서보모터로 조향합니다.

### Connection Diagram
![전원 연결](https://user-images.githubusercontent.com/89852937/233827449-28c3ea24-6f05-4a44-8653-4b7690fffd7a.png)

7.2V 배터리와 모터드라이버, DC-DC 강압 컨버터에 연결한 후 출력된 5V로 Jetson Nano에 연결합니다. 이후 Jetson Nano로 Lidar와 MCU에 USB 연결로 전원을 공급하고 제어합니다.

![Motor Drive 연결](https://user-images.githubusercontent.com/89852937/233826525-65ce1dd6-021c-40ff-987d-bfe4fbe22d95.png)

![Encoder 연결](https://user-images.githubusercontent.com/89852937/233826551-bbdb0dd9-8b33-4366-8421-31d2ccc3a24d.png)

![MPU 연결](https://user-images.githubusercontent.com/89852937/233826480-4d225b2e-3b03-4afc-b0f0-382d2861dc6f.png)

MPU9250 센서 모듈에 5V 전압을 인가하고 Arduino Due의 SDA 핀과 SCL 핀을 연결시켜 I2C 통신으로 제어합니다.
MPU9250은 로봇의 정중앙에 고정시켜 사용합니다.

## 로봇 테스트

### 1. Micro-ROS agent 실행

Dummy1은 Twist 메시지를 수신하여 제어할 수 있으며, MCU에서 수신한 메시지로부터 Odometry, IMU 데이터를 계산해 발행할 수 있습니다.

Agent 실행 :
```
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0
```

### 2. Move around

dummy1_bringup 패키지에 수록된 dummy1_teleop_twist_keyboard 로 키보드를 통해 동작할 수 있습니다.

```
ros2 run dummy1_bringup teleop
```


