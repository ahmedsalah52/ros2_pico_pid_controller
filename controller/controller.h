// controller.cpp
#pragma once


class Wheel_controller {
private:
    float kp,ki,kd;
    char pin_A, pin_B, pwm_pin, dir_pin;
    long long steps_counter = 0;
    long long steps_rate = 0;
    
    
public:
    Wheel_controller(); // Default constructor
    Wheel_controller(float p,float i, float d, char pin_A, char pin_B, char pwm_pin, char dir_pin); // Parameterized constructor
    void Interrupt_A(uint32_t events);
    void Interrupt_B(uint32_t events);
    // Getters
    long long getSteps() const;
    long double getStepsRate() const;
    char getPin_A() const;
    char getPin_B() const;
    char getPwm_pin() const;
    char getDir_pin() const;
    
};
