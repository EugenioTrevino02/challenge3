#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float32.h>

rcl_subscription_t subscriber;
rcl_publisher_t raw_pot_publisher;
rcl_publisher_t voltage_publisher;
std_msgs__msg__Float32 msg_float_1;
std_msgs__msg__Float32 msg_float_2;
std_msgs__msg__Int32 msg_int;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer_1;
rcl_timer_t timer_2;

int pot_val = 0;
float voltage = 0;
float pwm = 0;

const int PWM_CHANNEL = 0;    
const int PWM_FREQ = 5000;     
const int PWM_RESOLUTION = 8;  

#define PWM_PIN 15
#define LED_ROJO_PIN 13
#define POTENTIOMETER_PIN 36

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void error_loop(){
  while(1){
    digitalWrite(LED_ROJO_PIN, !digitalRead(LED_ROJO_PIN));
    delay(100);
  }
}

// Create timer 1 callback
void timer1_callback(rcl_timer_t * timer_1, int64_t last_call_time)
{  
  RCLC_UNUSED(last_call_time);
  if (timer_1 != NULL) {
    pot_val = analogRead(POTENTIOMETER_PIN);
  }
  ledcWrite(PWM_CHANNEL, abs((float)(pwm * 255/100)));
}

// Create timer 2 callback
void timer2_callback(rcl_timer_t * timer_2, int64_t last_call_time)
{  
  RCLC_UNUSED(last_call_time);
  if (timer_2 != NULL) {
    msg_int.data = pot_val;
    RCSOFTCHECK(rcl_publish(&raw_pot_publisher, &msg_int, NULL));
    voltage = pot_val * (3.3 / 4095);  
    msg_float_1.data = voltage;
    RCSOFTCHECK(rcl_publish(&voltage_publisher, &msg_float_1, NULL));
  }
}

// Create subscription for pwm duty cycle
void subscription_callback(const void * msgin)
{  
  const std_msgs__msg__Float32 * msg = (const std_msgs__msg__Float32 *)msgin;
  pwm = msg->data;
}

void setup() {
  set_microros_transports();
  
  pinMode(LED_ROJO_PIN, OUTPUT);
  digitalWrite(LED_ROJO_PIN, HIGH);  

  ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(PWM_PIN, PWM_CHANNEL);
  
  delay(2000);

  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_esp32_node", "", &support));

  // create publisher raw data
  RCCHECK(rclc_publisher_init_default(
    &raw_pot_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "/raw_pot"));

  // create publisher voltage data
  RCCHECK(rclc_publisher_init_default(
    &voltage_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "/voltage"));

  // create timer 1
  const unsigned int timer1_timeout = 10;
  RCCHECK(rclc_timer_init_default(
    &timer_1,
    &support,
    RCL_MS_TO_NS(timer1_timeout),
    timer1_callback));

  // create timer 2
  const unsigned int timer2_timeout = 100;
  RCCHECK(rclc_timer_init_default(
    &timer_2,
    &support,
    RCL_MS_TO_NS(timer2_timeout),
    timer2_callback));

  // create subscriber
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "/pwm_duty_cycle"));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer_1));
  RCCHECK(rclc_executor_add_timer(&executor, &timer_2));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg_float_2, &subscription_callback, ON_NEW_DATA));

}

void loop() {
  delay(100);
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}