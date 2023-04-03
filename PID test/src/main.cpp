#include <Arduino.h>

#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/float64.h>

#include "Calculates.h"
#include "PID.h"
#include "Motor.h"

rcl_subscription_t twist_sub;
rcl_subscription_t p_sub;
rcl_subscription_t i_sub;
rcl_subscription_t d_sub;

rcl_publisher_t ecd_rpm_pub;
rcl_publisher_t calc_rpm_pub;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t control_timer;

geometry_msgs__msg__Twist twist_msg;

std_msgs__msg__Float64 ecd_rpm_msg;
std_msgs__msg__Float64 calc_rpm_msg;

std_msgs__msg__Float64 p_msg;
std_msgs__msg__Float64 i_msg;
std_msgs__msg__Float64 d_msg;

unsigned long long time_offset = 0;
unsigned long prev_cmdvel_time = 0;

enum states
{
  Waiting_agent,
  agent_available,
  agent_connected,
  agent_disconnected
} state;

double wheel_diameter = 0.073; //M
double wheel_distence_x = 0.173; //M
double gear_ratio = 0.3513;
double encoder_gear_ratio = 0.7115;

#define max_rpm 176
#define motor_pin_R 2
#define motor_pin_L 3
#define servo_pin 4
#define motor_pin_R_EN 5
#define motor_pin_L_EN 6

//encoder config
#define pA 10
#define pB 11
#define pZ 12

#define Count_per_Revolution 6000

volatile signed long cnt_ = 0l;
volatile signed char dir_ = 1;

unsigned long prev_count_time = 0.0l;
long prev_count_tick = 0.0l;
double prev_encoder_vel = 0.0;

//PID config
double kp = 1.1;
double ki = 0.0;
double kd = 1.0;
int min_val = -255;
int max_val = 255;

//이동평균필터
const int numReadings = 5;
double readings[numReadings];
int readindex = 0.0;
double total = 0.0;

PID pid(min_val, max_val, kp, ki, kd);
control motor(motor_pin_R, motor_pin_L, motor_pin_R_EN, motor_pin_L_EN, servo_pin);
Calculates calculates(max_rpm, gear_ratio, encoder_gear_ratio, wheel_diameter, wheel_distence_x);

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
  dir_ = (digitalRead(pB) == HIGH)? -1: 1;
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
  double current_encoder_vel = (60.0 * delta_tick / (Count_per_Revolution * encoder_gear_ratio )) / dtm;
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

void subpvel_callback(const void *msgin)
{
  const std_msgs__msg__Float64 * msg = (const std_msgs__msg__Float64 *)msgin;
  p_msg.data = msg->data;
  pid.updatepvel(p_msg.data);
}

void subivel_callback(const void *msgin)
{
  const std_msgs__msg__Float64 * msg = (const std_msgs__msg__Float64 *)msgin;
  i_msg.data = msg->data;
  pid.updateivel(i_msg.data);
}

void subdvel_callback(const void *msgin)
{
  const std_msgs__msg__Float64 * msg = (const std_msgs__msg__Float64 *)msgin;
  d_msg.data = msg->data;
  pid.updatepvel(d_msg.data);
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
  calc_rpm_msg.data = calc_dc_rpm;
  ecd_rpm_msg.data = ecd_rpm;
  double pidvel = pid.pidcompute(calc_dc_rpm, ecd_rpm);
  float req_anguler_vel_z = twist_msg.angular.z;
  double motor_vel = MA_filter(pidvel);
  motor.run(motor_vel);
  float current_steering_angle = motor.steer(req_anguler_vel_z);
}

void publishData()
{
  RCSOFTCHECK(rcl_publish(&ecd_rpm_pub, &ecd_rpm_msg, NULL));
  RCSOFTCHECK(rcl_publish(&calc_rpm_pub, &calc_rpm_msg, NULL));

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

  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  rcl_node_options_t node_ops = rcl_node_get_default_options();
  node_ops.domain_id = 7;
  RCCHECK(rclc_node_init_with_options(&node, "Dummt1_PIDsetting_node", "", &support, &node_ops));

  RCCHECK(rclc_subscription_init_default(
    &twist_sub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "cmd_vel"
  ));

  RCCHECK(rclc_subscription_init_default(
    &p_sub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64),
    "p_vel"
  ));

  RCCHECK(rclc_subscription_init_default(
    &i_sub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64),
    "i_vel"
  ));

  RCCHECK(rclc_subscription_init_default(
    &d_sub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64),
    "d_vel"
  ));

  RCCHECK(rclc_publisher_init_default(
    &ecd_rpm_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64),
    "Encoder_vel"
  ));

  RCCHECK(rclc_publisher_init_default(
    &calc_rpm_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64),
    "Goal_vel"
  ));

  const unsigned int timeout = 20;
  RCCHECK(rclc_timer_init_default(
    &control_timer,
    &support,
    RCL_MS_TO_NS(timeout),
    controlcallback
  ));

  executor = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor, &support.context, 5, &allocator));

  RCCHECK(rclc_executor_add_subscription(
    &executor,
    &twist_sub,
    &twist_msg,
    &subcmdvel_callback,
    ON_NEW_DATA
  ));

  RCCHECK(rclc_executor_add_subscription(
    &executor,
    &p_sub,
    &p_msg,
    &subpvel_callback,
    ON_NEW_DATA
  ));

  RCCHECK(rclc_executor_add_subscription(
    &executor,
    &i_sub,
    &i_msg,
    &subivel_callback,
    ON_NEW_DATA
  ));

  RCCHECK(rclc_executor_add_subscription(
    &executor,
    &d_sub,
    &d_msg,
    &subdvel_callback,
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

  rcl_publisher_fini(&ecd_rpm_pub, &node);
  rcl_publisher_fini(&calc_rpm_pub, &node);
  rcl_subscription_fini(&twist_sub, &node);
  rcl_subscription_fini(&p_sub, &node);
  rcl_subscription_fini(&i_sub, &node);
  rcl_subscription_fini(&d_sub, &node);
  rclc_executor_fini(&executor);
  rclc_support_fini(&support);
  rcl_timer_fini(&control_timer);
  rcl_node_fini(&node);
  return true;
}
void setup() {
  Serial.begin(115200);
  state = Waiting_agent;
  pinMode(13, OUTPUT);

  attachInterrupt(pA, encoderCount, FALLING);
  pinMode(pB, INPUT);
  attachInterrupt(pZ, encoderReset, FALLING);

  set_microros_transports();
}

void loop() {
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
