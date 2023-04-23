
//Motor config
#define Max_rpm 176
#define Motor_pin_R 2
#define Motor_pin_L 3
#define Servo_pin 4
#define Motor_pin_R_EN 5
#define Motor_pin_L_EN 6

//Robot spec
#define Wheel_diameter 0.074
#define Wheel_distance_x 0.173
#define Gear_ratio 0.3513 // 모터에 부착된 기어

/*
        Front       __
    Wheel1  Wheel2    ㅣ
                       }  wheel_distance_x (m)
    Wheel3  Wheel4  __ㅣ
        Back


*/

//Encoder config
#define Encoder_gear_ratio 0.7115 // 엔코더에 부착된 기어
#define PA 10
#define PB 11
#define PZ 12
#define Count_per_Revolution 6000 // 2체배 방식이기 때문에 PPR * 2

//PID config
#define Kp 1.1
#define Ki 0.0
#define Kd 1.0
#define Min_val -254
#define Max_val 255

