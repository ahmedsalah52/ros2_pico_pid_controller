#include "pico_controller.h"

Robot_controller Robot;

rcl_publisher_t  odom_publisher;
nav_msgs__msg__Odometry odom_msg , prev_odom_msg;

std_msgs__msg__Int32 L_wheel_counter_msg,R_wheel_counter_msg;
std_msgs__msg__Float64 LinearVel_msg ,AngularVel_msg;

rcl_subscription_t twist_subscriber;
geometry_msgs__msg__Twist cmd_vel_msg;

rcl_clock_t clock;
rcl_allocator_t allocator = rcl_get_default_allocator();
rcl_ret_t rc = rcl_clock_init(RCL_SYSTEM_TIME, &clock, &allocator);

double set_LinearVel  = 0.0;
double set_AngularVel = 0.0;
long double prev_theta = 0.0;

rcl_publisher_t Left_Wheel_pub,Right_Wheel_pub;


int main()
{

    struct repeating_timer PID_timer;

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
    
    

    // Initialize a repeating timer that calls the callback function every 1000ms (1 second)
    add_repeating_timer_ms(PID_dt, PID_timer_callback, NULL, &PID_timer);
    
    gpio_set_irq_enabled_with_callback(Robot.getLWheelPin_A() , GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL , true, &Interrupt_funtion);
    gpio_set_irq_enabled_with_callback(Robot.getLWheelPin_B() , GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL , true, &Interrupt_funtion);
    gpio_set_irq_enabled_with_callback(Robot.getRWheelPin_A() , GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL , true, &Interrupt_funtion);
    gpio_set_irq_enabled_with_callback(Robot.getRWheelPin_B() , GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL , true, &Interrupt_funtion);

   
    
    rcl_timer_t timer;
    rcl_node_t node;
    rcl_allocator_t allocator;
    rclc_support_t support;
    rclc_executor_t executor, executor_sub;

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
    // rclc_publisher_init_default(
    //     &Left_Wheel_pub,
    //     &node,
    //     ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    //     "L_wheel_counter");
    // rclc_publisher_init_default(
    //     &Right_Wheel_pub,
    //     &node,
    //     ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    //     "R_wheel_counter");
    rclc_publisher_init_default(
    &odom_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
    "odom");
   
    rclc_timer_init_default(
        &timer,
        &support,
        RCL_MS_TO_NS(Odom_dt),
        publish_timer_callback);

    rclc_executor_init(&executor, &support.context, 2, &allocator);
    rclc_executor_add_subscription(&executor, &twist_subscriber, &cmd_vel_msg, &twist_callback, ON_NEW_DATA);
    rclc_executor_add_timer(&executor, &timer);


    // Initialize subscriber
    rclc_subscription_init_default(
        &twist_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "cmd_vel");

    // Add subscriber to executor
    //rclc_executor_init(&executor, &support.context, 2, &allocator); // Now managing 2 handles: a timer and a subscription

    LinearVel_msg.data = 0;
    AngularVel_msg.data = 0;

    while (true)
    {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
    }
    return 0;
}

void Interrupt_funtion(uint gpio, uint32_t events)
{
   Robot.Interrupt_function(gpio, events);
}
bool PID_timer_callback(struct repeating_timer *t) {
    // Do something
    Robot.update_Velocity(set_LinearVel, set_AngularVel);
    return true; // Return true to keep the timer repeating, false to stop
}

void publish_timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    

    rcl_time_point_value_t now,prev_time;
    rcl_clock_get_now(&clock, &now);
    //get time now
    odom_msg.header.stamp.sec = now / 1e9;
    odom_msg.header.stamp.nanosec = now % (int64_t)1e9;
    //odom_msg.header.frame_id = "odom"; // Or your odometry frame ID.

    //odom_msg.child_frame_id = "base_link"; // Or your robot's frame ID.
    odom_msg.pose.pose.position.x = Robot.get_X_pos();
    odom_msg.pose.pose.position.y = Robot.get_Y_pos();
    odom_msg.pose.pose.position.z = 0.0;

    Quaternion q = EulerToQuaternion(0.0, 0.0, Robot.get_Theta());


    odom_msg.pose.pose.orientation.x = q.x;
    odom_msg.pose.pose.orientation.y = q.y;
    odom_msg.pose.pose.orientation.z = q.z;
    odom_msg.pose.pose.orientation.w = q.w;

    
    for(int i = 0; i<36; i++) {
    if(i == 0 || i == 7 || i == 14) {
      odom_msg.pose.covariance[i] = .01;
     }
     else if (i == 21 || i == 28 || i== 35) {
       odom_msg.pose.covariance[i] += 0.1;
     }
     else {
       odom_msg.pose.covariance[i] = 0;
     }
    }

    odom_msg.twist.twist.linear.x = (Robot.get_X_pos() - prev_odom_msg.pose.pose.position.x) / (now / 1e9);
    odom_msg.twist.twist.linear.y = (Robot.get_Y_pos() - prev_odom_msg.pose.pose.position.y) / (now / 1e9);
    odom_msg.twist.twist.linear.z  = 0.0;
    odom_msg.twist.twist.angular.x = 0.0;
    odom_msg.twist.twist.angular.y = 0.0;
    odom_msg.twist.twist.angular.z = (Robot.get_Theta() - prev_theta) / (now / 1e9);
    //odom_msg.twist.covariance =

    //odom_msg.twist.twist.linear.x  = LinearVel_msg.data;
    //odom_msg.twist.twist.angular.z = AngularVel_msg.data;


    //double linear_velocity_x  = Robot.get_Linear_Velocity(); // Linear velocity in m/s
    //double angular_velocity_z = Robot.get_Angular_Velocity(); // Angular velocity in rad/s

    //odom_msg.header.frame_id = "odom"; // Or your odometry frame ID
    //odom_msg.child_frame_id = "base_link"; // Or your robot's frame ID

    // Set the velocities
    //odom_msg.twist.twist.linear.x  = linear_velocity_x;
    //odom_msg.twist.twist.angular.z = angular_velocity_z;

    //for debugging
    //odom_msg.twist.twist.linear.x = set_LinearVel;
    //odom_msg.twist.twist.linear.y = Robot.Right_Wheel.get_Velocity();
    //odom_msg.twist.twist.linear.z = set_AngularVel;
    //odom_msg.twist.twist.angular.y = pid_output;

    prev_time = now;
    prev_odom_msg = odom_msg;
    prev_theta = Robot.get_Theta();
    // Publish the odometry message
    rcl_ret_t ret = rcl_publish(&odom_publisher, &odom_msg, NULL);
}


void twist_callback(const void *msg_in) {
    const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msg_in;
    // Now msg->linear.x and msg->angular.z contain the new velocities.
    // Apply these velocities to your robot's control logic here.
    set_LinearVel  = msg->linear.x;
    set_AngularVel = msg->angular.z;
    // For example, update your motor control variables or call a function to adjust speed
    //Robot.update_Velocity(set_LinearVel, set_AngularVel);
}