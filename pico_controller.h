#include <stdio.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float64.h>
#include <nav_msgs/msg/odometry.h>
#include <geometry_msgs/msg/twist.h>
#include <rmw_microros/rmw_microros.h>
#include <geometry_msgs/msg/pose_with_covariance_stamped.h>
#include <geometry_msgs/msg/transform_stamped.h>
#include <tf2_msgs/msg/tf_message.h>
#include <sensor_msgs/msg/imu.h>
#include "controller/controller.h"
#include "controller/config.h"
#include "controller/quaternion/quaternion.h"
#include "imu_GY521/gy_521.h"


extern "C" {
#include "pico/stdlib.h"
#include "pico_uart_transports.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"
#include "hardware/irq.h"
#include "hardware/pwm.h"
#include <rosidl_runtime_c/string_functions.h>
}


void Interrupt_funtion(uint gpio, uint32_t events);
bool PID_timer_callback(struct repeating_timer *t);
void publish_timer_callback(rcl_timer_t *timer, int64_t last_call_time);
void imu_publish_timer_callback(rcl_timer_t * timer, int64_t last_call_time);
void twist_callback(const void *msg_in);
void initialpos_callback(const void *msg_id);