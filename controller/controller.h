// controller.cpp
#pragma once

#include "PID/pid.h"
#include "config.h"
#include <stdio.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <rmw_microros/rmw_microros.h>
extern "C" {
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"
#include "hardware/irq.h"
#include "hardware/pwm.h"
}
#include <cmath>
#include "quaternion/quaternion.h"
class Wheel_controller : private PID{
private:
    char pin_A, pin_B, PWM_A, PWM_B;
    long long Ticks_counter = 0;
    long long prev_Ticks_counter = 0;

    double current_ticks_rate = 0;
    double dt;

public:
    Wheel_controller(); // Default constructor
    Wheel_controller(double Kp,double Ki, double Kd, double dt ,double min_v, double max_v,char pin_A, char pin_B, char PWM_A, char PWM_B); // Parameterized constructor

    void Interrupt_A(uint32_t events);
    void Interrupt_B(uint32_t events);
    // Getters
    long long getTicks() const;
    char getPin_A() const;
    char getPin_B() const;
    char getPwm_pin() const;
    char getDir_pin() const;
    long long getDelta_Ticks();
    double get_Velocity();

    // Setters
    double set_Velocity(double set_point);
    void set_PWM_pins(char pin_A,char pin_B);
    void set_PWM_values(char pin_A, char pin_B, double value_A, double value_B);
};



class Robot_controller{
private:
    
    Wheel_controller Left_Wheel;
    Wheel_controller Right_Wheel;
    double WheelBase, Wheel_Radius;

    long long prev_Left_Ticks = 0;
    long long prev_Right_Ticks = 0;

    long double X_pos = 0;
    long double Y_pos = 0;
    long double theta = 0;

public:
    Robot_controller();
    
    void Interrupt_function(uint gpio, uint32_t events);

    void Set_X_pos(long double x);
    void Set_Y_pos(long double y);
    void Set_Theta(long double t);
    
    double get_Linear_Velocity();
    double get_Angular_Velocity();
    double update_Velocity(double linear, double angular);

    long long get_Left_Ticks();
    long long get_Right_Ticks();
    char getLWheelPin_A();
    char getLWheelPin_B();
    char getRWheelPin_A();
    char getRWheelPin_B();
    void update_Odom();
    long double get_X_pos();
    long double get_Y_pos();
    long double get_Theta();
};
