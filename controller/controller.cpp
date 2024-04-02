// controller.cpp

#include "controller.h"




Wheel_controller::Wheel_controller()
{
    
}
Wheel_controller::Wheel_controller(double Kp,double Ki, double Kd, double dt ,double min_v, double max_v,char pin_A, char pin_B, char PWM_A, char PWM_B)
:PID( dt, max_v, min_v, Kp, Kd, Ki )
{
    this->pin_A = pin_A;
    this->pin_B = pin_B;
    this->PWM_A = PWM_A;
    this->PWM_B = PWM_B;
    this->dt=dt;

    //pinouts
    gpio_init(this->pin_A);
    gpio_init(this->pin_B);

    gpio_set_dir(this->pin_A, GPIO_IN);
    gpio_set_dir(this->pin_B, GPIO_IN);

    gpio_pull_up(this->pin_A);
    gpio_pull_up(this->pin_B);

    //output pwm pins
    set_PWM_pins(this->PWM_A, this->PWM_B);
    set_PWM_values(this->PWM_A, this->PWM_B, 0, 0);
}

// Getters
long long   Wheel_controller::getTicks() const { return this->Ticks_counter; }
char        Wheel_controller::getPin_A() const { return this->pin_A; }
char        Wheel_controller::getPin_B() const { return this->pin_B; }
long long Wheel_controller::getDelta_Ticks()
{ 
    this->current_ticks_rate = static_cast<double>(this->Ticks_counter - this->prev_Ticks_counter)/this->dt;
    this->prev_Ticks_counter = this->Ticks_counter;
    return this->current_ticks_rate;
}

double Wheel_controller::get_Velocity()
{
    return (this->current_ticks_rate* wheel_radius * 2 * M_PI)/TicksPerRevolution;
}


double Wheel_controller::set_Velocity(double set_point)
{
    double ticks_rate = this->getDelta_Ticks();
    
    double current_velocity = (ticks_rate* wheel_radius * 2 * M_PI)/TicksPerRevolution;

    
    double output = calculate(set_point, current_velocity);
    
    if(output > 0)
    {   
        set_PWM_values(this->PWM_A, this->PWM_B, output, 0);
    }
    else
    {
        set_PWM_values(this->PWM_A, this->PWM_B, 0     , abs(output));
    }

    return output;
}
void Wheel_controller::set_PWM_pins(char pin_A,char pin_B)
{
    gpio_set_function(pin_A, GPIO_FUNC_PWM);
    uint slice_num_A = pwm_gpio_to_slice_num(pin_A);
    pwm_set_enabled(slice_num_A, true);
    pwm_set_wrap(slice_num_A, 500);

    gpio_set_function(pin_B, GPIO_FUNC_PWM);
    uint slice_num_B = pwm_gpio_to_slice_num(pin_B);
    pwm_set_enabled(slice_num_B, true);
    pwm_set_wrap(slice_num_B, 500);
}

void Wheel_controller::set_PWM_values(char pin_A, char pin_B, double value_A, double value_B)
{   
    value_A = value_A * (double)counter_res / (double)output_res;
    value_B = value_B * (double)counter_res / (double)output_res;

    uint slice_num_A = pwm_gpio_to_slice_num(pin_A);
    uint slice_num_B = pwm_gpio_to_slice_num(pin_B);

    pwm_set_chan_level(slice_num_A, PWM_CHAN_A, 0);
    pwm_set_chan_level(slice_num_B, PWM_CHAN_B, 0);


    pwm_set_chan_level(slice_num_A, PWM_CHAN_A, value_A);
    pwm_set_chan_level(slice_num_B, PWM_CHAN_B, value_B);

}

// Interrupt handler
void Wheel_controller::Interrupt_A(uint32_t events)
{
    if(events == GPIO_IRQ_EDGE_RISE)
    {
        if (gpio_get(this->pin_B))
        {
            this ->Ticks_counter++;
        }
        else
        {
            this ->Ticks_counter--;
        }
    }
    else if(events == GPIO_IRQ_EDGE_FALL)
    {
        if (gpio_get(this->pin_B))
        {
            this ->Ticks_counter--;
        }
        else
        {
            this ->Ticks_counter++;
        }
    }
}

void Wheel_controller::Interrupt_B(uint32_t events)
{
    if(events == GPIO_IRQ_EDGE_RISE)
    {
        if (gpio_get(this->pin_A))
        {
            this ->Ticks_counter--;
        }
        else
        {
            this ->Ticks_counter++;
        }
    }
    else if(events == GPIO_IRQ_EDGE_FALL)
    {
        if (gpio_get(this->pin_A))
        {
            this ->Ticks_counter++;
        }
        else
        {
            this ->Ticks_counter--;
        }
    } 
}



Robot_controller::Robot_controller():
WheelBase(wheelbase), 
Wheel_Radius(wheel_radius), 
Left_Wheel(L_Kp  ,L_Ki ,L_Kd  , PID_dt_sec ,L_min, L_max, Left_Encoder_A_Pin  , Left_Encoder_B_Pin  , Left_Motor_PWM_A  , Left_Motor_PWM_B),
Right_Wheel(R_Kp ,R_Ki ,R_Kd  , PID_dt_sec ,R_min, R_max, Right_Encoder_A_Pin , Right_Encoder_B_Pin , Right_Motor_PWM_A , Right_Motor_PWM_B)
{
  
}

char Robot_controller::getLWheelPin_A(){ return this->Left_Wheel.getPin_A(); }
char Robot_controller::getLWheelPin_B(){ return this->Left_Wheel.getPin_B(); }
char Robot_controller::getRWheelPin_A(){ return this->Right_Wheel.getPin_A(); }
char Robot_controller::getRWheelPin_B(){ return this->Right_Wheel.getPin_B(); }

double Robot_controller::update_Velocity(double linear, double angular)
{
    double left_v  = linear - angular * this->WheelBase / 2;
    double right_v = linear + angular * this->WheelBase / 2;
    double output_l = this->Left_Wheel.set_Velocity(left_v);
    double output_r = this->Right_Wheel.set_Velocity(right_v);
    return output_l;
}

double Robot_controller::get_Linear_Velocity()
{
    double left_v  = this->Left_Wheel.get_Velocity();
    double right_v = this->Right_Wheel.get_Velocity();
    return (left_v + right_v) / 2;

}

double Robot_controller::get_Angular_Velocity()
{
    double left_v  = this->Left_Wheel.get_Velocity();
    double right_v = this->Right_Wheel.get_Velocity();
    return (right_v - left_v) / this->WheelBase;
}

long long Robot_controller::get_Left_Ticks()
{
    return this->Left_Wheel.getTicks();
}

long long Robot_controller::get_Right_Ticks()
{
    return this->Right_Wheel.getTicks();
}
void Robot_controller::Interrupt_function(uint gpio, uint32_t events)
{
     if(gpio == Left_Wheel.getPin_A())
    {
        Left_Wheel.Interrupt_A(events);
    }
    else if(gpio == Left_Wheel.getPin_B())
    {
        Left_Wheel.Interrupt_B(events);
    }
    else if(gpio == Right_Wheel.getPin_A())
    {
        Right_Wheel.Interrupt_A(events);
    }
    else if(gpio == Right_Wheel.getPin_B())
    {
        Right_Wheel.Interrupt_B(events);
    }
}


void Robot_controller::update_Odom()
{
    long long right_ticks = this->Right_Wheel.getTicks();
    long long left_ticks  = this->Left_Wheel.getTicks();


    long double distance_left  = (long double)(left_ticks  - this->prev_Left_Ticks)  * 2.0 * M_PI * wheel_radius / TicksPerRevolution;
    long double distance_right = (long double)(right_ticks - this->prev_Right_Ticks) * 2.0 * M_PI * wheel_radius / TicksPerRevolution;
    
    // Calculate the change in orientation
    long double cycleAngle = (distance_right - distance_left) / wheelbase;
    
    // Assuming small delta_theta, calculate approximate linear displacement
    long double cycleDistance = (distance_left + distance_right) / 2.0;
    
    //long double delta_x = cycleDistance * cos(cycleAngle);
    //long double delta_y = cycleDistance * sin(cycleAngle);
    long double delta_x = cycleDistance * cos(this->theta + cycleAngle / 2);
    long double delta_y = cycleDistance * sin(this->theta + cycleAngle / 2);

    long double delta_theta = cycleAngle;
    
    this->X_pos += delta_x;
    this->Y_pos += delta_y;
    this->theta += delta_theta;

    
    // Normalize theta to the range [-pi, pi]
    // this->theta = fmod(this->theta, 2.0 * M_PI);
    // if (this->theta < 0)
    //     this->theta += 2.0 * M_PI;
    // this->theta -= M_PI;
    if (this->theta > M_PI)
        this->theta -= 2.0 * M_PI;
    else if (this->theta < -M_PI)
        this->theta += 2.0 * M_PI;


    // Update previous ticks
    this->prev_Left_Ticks  = left_ticks;
    this->prev_Right_Ticks = right_ticks;
}
void Robot_controller::Set_X_pos(long double x)
{
    this->X_pos = x;
}
void Robot_controller::Set_Y_pos(long double y)
{
    this->Y_pos = y;
}
void Robot_controller::Set_Theta(long double t)
{
    this->theta = t;
}

long double Robot_controller::get_X_pos()
{
    return this->X_pos;
}
long double Robot_controller::get_Y_pos()
{
    return this->Y_pos;
}
long double Robot_controller::get_Theta()
{
    return this->theta;
}


