// controller.cpp
#include <stdio.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <rmw_microros/rmw_microros.h>


#include "controller.h"
#include "pico/stdlib.h"
#include "hardware/gpio.h"

// Default constructor
//Wheel_controller::Wheel_controller() : kp(1.0), kd(1.0), ki(1.0) {}

// Parameterized constructor
Wheel_controller::Wheel_controller(float p,float i, float d, char pin_A, char pin_B, char pwm_pin, char dir_pin)
{
    this->kp = p;
    this->ki = i;
    this->kd = d;
    this->pin_A = pin_A;
    this->pin_B = pin_B;
    this->pwm_pin = pwm_pin;
    this->dir_pin = dir_pin;
    
    gpio_init(this->pin_A);
    gpio_init(this->pin_B);

    gpio_set_dir(this->pin_A, GPIO_IN);
    gpio_set_dir(this->pin_B, GPIO_IN);

    gpio_pull_up(this->pin_A);
    gpio_pull_up(this->pin_B);
   

}

// Getters
long long   Wheel_controller::getSteps()     const { return this->steps_counter; }
long double Wheel_controller::getStepsRate() const { return this->steps_rate; }
char   Wheel_controller::getPin_A()          const { return this->pin_A; }
char   Wheel_controller::getPin_B()          const { return this->pin_B; }

// Interrupt handler
void Wheel_controller::Interrupt_A(uint32_t events)
{
    if(events == GPIO_IRQ_EDGE_RISE)
    {
        if (gpio_get(this->pin_B))
        {
            this ->steps_counter++;
        }
        else
        {
            this ->steps_counter--;
        }
    }
    else if(events == GPIO_IRQ_EDGE_FALL)
    {
        if (gpio_get(this->pin_B))
        {
            this ->steps_counter--;
        }
        else
        {
            this ->steps_counter++;
        }
    }
}

void Wheel_controller::Interrupt_B(uint32_t events)
{
    if(events == GPIO_IRQ_EDGE_RISE)
    {
        if (gpio_get(this->pin_A))
        {
            this ->steps_counter--;
        }
        else
        {
            this ->steps_counter++;
        }
    }
    else if(events == GPIO_IRQ_EDGE_FALL)
    {
        if (gpio_get(this->pin_A))
        {
            this ->steps_counter++;
        }
        else
        {
            this ->steps_counter--;
        }
    } 
}

