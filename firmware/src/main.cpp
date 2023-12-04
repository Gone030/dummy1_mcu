#include <Arduino.h>

#include <micro_ros_arduino.h>
#include <rcl/rcl.h> // ROS Client Library for C
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>
#include <std_msgs/msg/float64.h>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/joint_state.h>

#include <Dummy1_config.h>
#include "Imu.h"
#include "Calculates.h"
#include "Motor.h"
#include "PID.h"
#include "Odometry.h"

rcl_subscription_t twist_sub;

rcl_publisher_t odom_pub;
rcl_publisher_t imu_pub;
rcl_publisher_t steer_pub;

// rclc 에서는 executor, support, allocator를 선언 해야 함
rclc_executor_t executor; // 임베디드 환경에서 동적 메모리의 할당, 해제
rclc_support_t support; // ROS 클라이언트의 생성, rcl_node, rcl_subscription, rcl_timer, rclc_executor 생성 단순화
rcl_allocator_t allocator; // subscription timer callback

rcl_node_t node;
rcl_timer_t control_timer;


geometry_msgs__msg__Twist twist_msg;
nav_msgs__msg__Odometry odom_msg;
sensor_msgs__msg__JointState jointstate_msg;
sensor_msgs__msg__Imu imu_msg;
std_msgs__msg__Float64 steer_msg;

unsigned long long time_offset = 0;
unsigned long prev_cmdvel_time = 0;
unsigned long prev_odom_update = 0;

enum states
{
  Waiting_agent,
  agent_available,
  agent_connected,
  agent_disconnected
} state;

volatile signed long cnt_ = 0l;
volatile signed char dir_ = 1;

unsigned long prev_count_time = 0.0l;
long prev_count_tick = 0.0l;
double prev_encoder_vel = 0.0;

//이동평균필터
const int numReadings = 5;
double readings[numReadings];
int readindex = 0.0;
double total = 0.0;

PID pid(Min_val, Max_val, Kp, Ki, Kd);
control motor(Motor_pin_R, Motor_pin_L, Motor_pin_R_EN, Motor_pin_L_EN, Servo_pin);
Calculates calculates(Max_rpm, Gear_ratio, Encoder_gear_ratio, Wheel_diameter, Wheel_distance_x);
Imu imu;
Odom odom;

void error_loop(){
  while(1){
    for(int i=0; i<2; i++)
    {
      digitalWrite(13, HIGH);
      delay(150);
      digitalWrite(13, LOW);
      delay(150);
    }
    delay(1000);
  }
}

// ROS 클라이언트가 제대로 생성돼있는지 검사
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}
#define EXECUTE_EVERY_N_NS(MS, X) do{ \
 static volatile int64_t init = -1; \
 if (init == -1) { init = uxr_millis();} \
 if (uxr_millis() - init > MS) {X; init = uxr_millis();} \
} while (0)


void syncTime()
{
  unsigned long now = millis();
  RCCHECK(rmw_uros_sync_session(10));
  unsigned long long ros_time_ms = rmw_uros_epoch_millis();
  time_offset = ros_time_ms - now;
}
struct timespec getTime()
{
  struct timespec tp = {0};
  unsigned long long now = millis() + time_offset;
  tp.tv_sec = now / 1000;
  tp.tv_nsec = (now % 1000) * 1000000;

  return tp;
}

void encoderCount()
{
  dir_ = (digitalRead(PB) == HIGH)? -1: 1; // 엔코더의 방향이 반대로 설치됨
  cnt_ += dir_;
}
void encoderReset(){ cnt_ = 0; }
long returnCount(){ return cnt_; }
double getRPM()
{
  long current_tick = returnCount();
  unsigned long current_time = micros();
  unsigned long dt = current_time - prev_count_time;
  double delta_tick = (double)(current_tick - prev_count_tick);

  double dtm = (double)dt / 1000000.0;

  prev_count_tick = current_tick;
  prev_count_time = current_time;
  double current_encoder_vel = (60.0 * delta_tick / (Count_per_Revolution * Encoder_gear_ratio )) / dtm;
  if(abs(prev_encoder_vel - current_encoder_vel) > 200.0)
  {
    return prev_encoder_vel;
  }
  else
  {
    prev_encoder_vel = current_encoder_vel;
    return current_encoder_vel;
  }
}

void subcmdvel_callback(const void *msgin)
{
  digitalWrite(13, !digitalRead(13));
  prev_cmdvel_time = millis();
}

double MA_filter(double vel)
{
  total = total - readings[readindex];
  readings[readindex] = vel;
  total = total + readings[readindex];
  readindex++;
  if (readindex >= numReadings) { readindex = 0; }
  double average = total / numReadings;
  return average;
}

void move()
{
  if((millis() - prev_cmdvel_time) >= 200)
  {
    twist_msg.linear.x = 0.0;
    twist_msg.angular.z = 0.0;
  }
  double calc_dc_rpm = calculates.CalculateRpm(twist_msg.linear.x);
  double ecd_rpm = getRPM();
  double pidvel = pid.pidcompute(calc_dc_rpm, ecd_rpm);
  float req_angular_vel_z = twist_msg.angular.z;
  double motor_vel = MA_filter(pidvel);
  motor.run(motor_vel);
  float current_steering_angle = motor.steer(req_angular_vel_z);
  steer_msg.data = current_steering_angle;
  Calculates::vel current_vel = calculates.get_velocities(current_steering_angle, ecd_rpm);

  unsigned long now = millis();
  float dt = (now  - prev_odom_update) / 1000.0;
  prev_odom_update = now;
  odom.update(dt, current_vel.linear_x , current_vel.angular_z);
}

void publishData()
{
  imu_msg = imu.getdata();
  odom_msg = odom.getOdomData();

  struct timespec time_stamp = getTime();

  odom_msg.header.stamp.sec = time_stamp.tv_sec;
  odom_msg.header.stamp.nanosec = time_stamp.tv_nsec;
  imu_msg.header.stamp.sec = time_stamp.tv_sec;
  imu_msg.header.stamp.nanosec = time_stamp.tv_nsec;

  RCSOFTCHECK(rcl_publish(&imu_pub, &imu_msg, NULL));
  RCSOFTCHECK(rcl_publish(&odom_pub, &odom_msg, NULL));
  RCSOFTCHECK(rcl_publish(&steer_pub, &steer_msg, NULL));
}

void controlcallback(rcl_timer_t *timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);
  if(timer != NULL)
  {
    move();
    publishData();
  }
}

bool createEntities()
{
  allocator = rcl_get_default_allocator();

  rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();

  RCCHECK(rcl_init_options_init(&init_options, allocator));
  // rmw_init_options_t* rmw_options = rcl_init_options_get_rmw_init_options(&init_options);

  size_t domain_id = 7;
  RCCHECK(rcl_init_options_set_domain_id(&init_options, domain_id));

  RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

  RCCHECK(rclc_node_init_default(&node, "Dummy1_Due_node", "", &support));

  RCCHECK(rclc_subscription_init_default(
    &twist_sub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "cmd_vel"
  ));

  RCCHECK(rclc_publisher_init_default(
    &odom_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
    "odom/unfiltered"
  ));

  RCCHECK(rclc_publisher_init_default(
    &imu_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
    "imu/data"
  ));

  RCCHECK(rclc_publisher_init_default(
    &steer_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64),
    "steer_angle"
  ))

  const unsigned int timeout = 20;
  RCCHECK(rclc_timer_init_default(
    &control_timer,
    &support,
    RCL_MS_TO_NS(timeout),
    controlcallback
  ));

  executor = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator)); //(subscriber + timer)

  RCCHECK(rclc_executor_add_subscription(
    &executor,
    &twist_sub,
    &twist_msg,
    &subcmdvel_callback,
    ON_NEW_DATA
  ));

  RCCHECK(rclc_executor_add_timer(&executor, &control_timer));

  syncTime();
  return true;
}

bool destroyEntities()
{
  rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
  (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  rcl_publisher_fini(&imu_pub, &node);
  rcl_publisher_fini(&odom_pub, &node);
  rcl_publisher_fini(&steer_pub, &node);
  rcl_subscription_fini(&twist_sub, &node);
  rclc_executor_fini(&executor);
  rclc_support_fini(&support);
  rcl_timer_fini(&control_timer);
  rcl_node_fini(&node);
  return true;
}
void setup()
{
  Serial.begin(115200);
  state = Waiting_agent;
  pinMode(13, OUTPUT);

  bool imu_setup = imu.init();
  if(!imu_setup)
  {
    while (1)
    {
      for(int i=0; i<3; i++)
      {
        digitalWrite(13, HIGH);
        delay(100);
        digitalWrite(13, LOW);
        delay(100);
      }
    delay(1000);
    }
  }

  attachInterrupt(PA, encoderCount, FALLING);
  pinMode(PB, INPUT);
  attachInterrupt(PZ, encoderReset, FALLING);

  set_microros_transports();
}

void loop()
{
  switch (state)
  {
    case Waiting_agent:
      EXECUTE_EVERY_N_NS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? agent_available : Waiting_agent;);
      break;
    case agent_available:
      state = (true == createEntities()) ? agent_connected : Waiting_agent;
      if ( state == Waiting_agent)
      {
        destroyEntities();
      }
      break;
    case agent_connected:
      EXECUTE_EVERY_N_NS(200, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? agent_connected : agent_disconnected;);
      if (state == agent_connected)
      {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
      }
      break;
    case agent_disconnected:
      destroyEntities();
      state = Waiting_agent;
      break;
    default:
      break;
  }
}
