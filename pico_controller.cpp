#include "pico_controller.h"


const uint LED_PIN = 25;

Wheel_controller Left_Wheel(1.0,0.0,0.0,16,17,25,26);
Wheel_controller Right_Wheel(1.0,0.0,0.0,14,15,25,26);

rcl_publisher_t Left_Wheel_pub,Right_Wheel_pub;
std_msgs__msg__Int32 L_wheel_counter_msg,R_wheel_counter_msg;

void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    rcl_ret_t ret_l = rcl_publish(&Left_Wheel_pub , &L_wheel_counter_msg, NULL);
    rcl_ret_t ret_r = rcl_publish(&Right_Wheel_pub, &R_wheel_counter_msg, NULL);

    L_wheel_counter_msg.data = Left_Wheel.getSteps();
    R_wheel_counter_msg.data = Right_Wheel.getSteps();

}

int main()
{
    rmw_uros_set_custom_transport(
		true,
		NULL,
		pico_serial_transport_open,
		pico_serial_transport_close,
		pico_serial_transport_write,
		pico_serial_transport_read
	);
    stdio_init_all();
    // Initialize PIN_X as input with pull-up resistor

    gpio_set_irq_enabled_with_callback(Left_Wheel.getPin_A()  , GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL , true, &Interrupt_funtion);
    gpio_set_irq_enabled_with_callback(Left_Wheel.getPin_B()  , GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL , true, &Interrupt_funtion);
    gpio_set_irq_enabled_with_callback(Right_Wheel.getPin_A() , GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL , true, &Interrupt_funtion);
    gpio_set_irq_enabled_with_callback(Right_Wheel.getPin_B() , GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL , true, &Interrupt_funtion);

   
    
    rcl_timer_t timer;
    rcl_node_t node;
    rcl_allocator_t allocator;
    rclc_support_t support;
    rclc_executor_t executor;

    allocator = rcl_get_default_allocator();

    // Wait for agent successful ping for 2 minutes.
    const int timeout_ms = 1000; 
    const uint8_t attempts = 120;

    rcl_ret_t ret = rmw_uros_ping_agent(timeout_ms, attempts);

    if (ret != RCL_RET_OK)
    {
        // Unreachable agent, exiting program.
        return ret;
    }

    rclc_support_init(&support, 0, NULL, &allocator);

    rclc_node_init_default(&node, "pico_node", "", &support);
    rclc_publisher_init_default(
        &Left_Wheel_pub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "L_wheel_counter");
    rclc_publisher_init_default(
        &Right_Wheel_pub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "R_wheel_counter");
   
    rclc_timer_init_default(
        &timer,
        &support,
        RCL_MS_TO_NS(1000),
        timer_callback);

    rclc_executor_init(&executor, &support.context, 1, &allocator);
    rclc_executor_add_timer(&executor, &timer);


    L_wheel_counter_msg.data = 0;
    R_wheel_counter_msg.data = 0;

    while (true)
    {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
    }
    return 0;
}

void Interrupt_funtion(uint gpio, uint32_t events)
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