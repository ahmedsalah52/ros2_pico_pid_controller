#include "pico_controller.h"

Robot_controller Robot;
IMU_MPU6050  imu_0(i2c0, 4, 5, -1, 0);

rcl_publisher_t  odom_publisher, imu_publisher;// , tf_publisher;
//tf2_msgs__msg__TFMessage tf_msg;
nav_msgs__msg__Odometry odom_msg , prev_odom_msg;
geometry_msgs__msg__PoseWithCovarianceStamped initialpose_msg;

sensor_msgs__msg__Imu imu_msg;

std_msgs__msg__Int32 L_wheel_counter_msg,R_wheel_counter_msg;
std_msgs__msg__Float64 LinearVel_msg ,AngularVel_msg;

rcl_subscription_t twist_subscriber , initialpose_subscriber;
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
    "/wheel/odometry");

    rclc_publisher_init_default(
    &imu_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
    "/sensor/imu0");

    //tf msg 
    // rclc_publisher_init_default(
    //     &tf_publisher,
    //     &node,
    //     ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, TransformStamped),
    //     "tf");

    rclc_timer_init_default(
        &timer,
        &support,
        RCL_MS_TO_NS(Odom_dt),
        publish_timer_callback);

    rclc_timer_init_default(
        &timer,
        &support,
        RCL_MS_TO_NS(IMU_dt),
        imu_publish_timer_callback);
    

    rclc_executor_init(&executor, &support.context, 3, &allocator);
    rclc_executor_add_subscription(&executor, &twist_subscriber, &cmd_vel_msg, &twist_callback, ON_NEW_DATA);
    rclc_executor_add_subscription(&executor, &initialpose_subscriber, &initialpose_msg, &initialpos_callback, ON_NEW_DATA);
    rclc_executor_add_timer(&executor, &timer);


    // Initialize subscriber
    rclc_subscription_init_default(
        &twist_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "cmd_vel");
    
    rclc_subscription_init_default(
        &initialpose_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, PoseWithCovarianceStamped),
        "initialpose");

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
    Robot.update_Odom();

    rcl_time_point_value_t now,prev_time;

    rcl_clock_get_now(&clock, &now);

    // double dt = (now - prev_time) / 1e9;
    double dt = (prev_time == 0) ? 0.0 : (now - prev_time) / 1e9; // Avoid division by zero in the first call

    // get data 
    long double x_pos = Robot.get_X_pos();
    long double y_pos = Robot.get_Y_pos();
    long double theta = Robot.get_Theta();
    //transform 
    // get time now
    odom_msg.header.stamp.sec = now / 1e9;
    odom_msg.header.stamp.nanosec = now % (int64_t)1e9;
    
    // frames
    bool success  = rosidl_runtime_c__String__assign(&odom_msg.header.frame_id, "odom");
    bool success2 = rosidl_runtime_c__String__assign(&odom_msg.child_frame_id, "base_link");
    //odom_msg.header.frame_id.data = "odom"; // Or your odometry frame ID.
    //odom_msg.child_frame_id.data  = "base_link"; // Or your robot's frame ID.
    
    // pose
    odom_msg.pose.pose.position.x = x_pos;
    odom_msg.pose.pose.position.y = y_pos;
    odom_msg.pose.pose.position.z = 0.0;

    // orientation
    Quaternion q = EulerToQuaternion(0.0, 0.0, theta);
    odom_msg.pose.pose.orientation.x = q.x;
    odom_msg.pose.pose.orientation.y = q.y;
    odom_msg.pose.pose.orientation.z = q.z;
    odom_msg.pose.pose.orientation.w = q.w;

    
    for(int i = 0; i<36; i++) {
    if(i == 0 || i == 7 || i == 14) {
      odom_msg.pose.covariance[i] = .01;
     }
     else if (i == 21 || i == 28 || i== 35) {
       odom_msg.pose.covariance[i] = 0.1;
     }
     else {
       odom_msg.pose.covariance[i] = 0;
     }
    }
    
    if(dt>0.0)
    {
        odom_msg.twist.twist.linear.x = (x_pos - prev_odom_msg.pose.pose.position.x) / (dt);
        odom_msg.twist.twist.linear.y = (y_pos - prev_odom_msg.pose.pose.position.y) / (dt);
        odom_msg.twist.twist.linear.z = 0.0;
        
        
        odom_msg.twist.twist.angular.x = 0.0;
        odom_msg.twist.twist.angular.y = 0.0;
        odom_msg.twist.twist.angular.z = (theta - prev_theta) / (dt);
    }
    else
    {
        odom_msg.twist.twist.linear.x = 0.0;
        odom_msg.twist.twist.linear.y = 0.0;
        odom_msg.twist.twist.linear.z = 0.0;
        
        odom_msg.twist.twist.angular.x = 0.0;
        odom_msg.twist.twist.angular.y = 0.0;
        odom_msg.twist.twist.angular.z = 0.0;
    }
    //odom_msg.twist.covariance = ;

    
    // Publish the odometry message
    rcl_ret_t ret = rcl_publish(&odom_publisher, &odom_msg, NULL);

    // // Publish the TF message
    // //frame transform
    // tf_msg.transforms.data->header.stamp.sec = now / 1e9;
    // tf_msg.transforms.data->header.stamp.nanosec = now % (int64_t)1e9;
    // //bool success = rosidl_runtime_c__String__assign(&tf_msg.transforms.data->header.frame_id.data, "odom");
    // success = rosidl_runtime_c__String__assign(&tf_msg.transforms.data->header.frame_id, "odom");
    // success2 = rosidl_runtime_c__String__assign(&tf_msg.transforms.data->child_frame_id, "base_link");

    // if (!success) {
    //     // Handle error
    // }
    
    // //tf_msg.transforms.data->header.frame_id.data = "odom";
    // //tf_msg.transforms.data->child_frame_id.data = "base_link";

    
    // 
    // tf_msg.transforms.data->transform.translation.x = x_pos;
    // tf_msg.transforms.data->transform.translation.y = y_pos;
    // tf_msg.transforms.data->transform.translation.z = 0.0;

    // tf_msg.transforms.data->transform.rotation.x = q.x;
    // tf_msg.transforms.data->transform.rotation.y = q.y;
    // tf_msg.transforms.data->transform.rotation.z = q.z;
    // tf_msg.transforms.data->transform.rotation.w = q.w;

    // rcl_ret_t ret2 = rcl_publish(&tf_publisher, &tf_msg, NULL);


    // update the prevous states
    prev_time = now;
    prev_odom_msg = odom_msg;
    prev_theta = Robot.get_Theta();
}


void imu_publish_timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
    rcl_time_point_value_t now;

    rcl_clock_get_now(&clock, &now);
    imu_msg.header.stamp.sec = now / 1e9;
    imu_msg.header.stamp.nanosec = now % (int64_t)1e9;

    bool success  = rosidl_runtime_c__String__assign(&imu_msg.header.frame_id , "imu_link");

    imu_raw_data readings = imu_0.get_readings();
    
    //linear acceleration and covariance
    imu_msg.linear_acceleration.x = readings.accel_x;
    imu_msg.linear_acceleration.y = readings.accel_y;
    imu_msg.linear_acceleration.z = readings.accel_z;

    imu_msg.angular_velocity.x = readings.gyro_x;
    imu_msg.angular_velocity.y = readings.gyro_y;
    imu_msg.angular_velocity.z = readings.gyro_z;

    //covariance
    //set orientation covariance to -1 to be ignored
    imu_msg.orientation_covariance[0] = -1;

    imu_msg.linear_acceleration_covariance[0] = linear_acc_var_x;
    imu_msg.linear_acceleration_covariance[4] = linear_acc_var_y;
    imu_msg.linear_acceleration_covariance[8] = linear_acc_var_z;

    imu_msg.angular_velocity_covariance[0] = angular_vel_var_x;
    imu_msg.angular_velocity_covariance[4] = angular_vel_var_y;
    imu_msg.angular_velocity_covariance[8] = angular_vel_var_z;

    rcl_ret_t ret = rcl_publish(&imu_publisher, &imu_msg, NULL);
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

void initialpos_callback(const void *msg_id)
{
    const geometry_msgs__msg__PoseWithCovarianceStamped *msg = (const geometry_msgs__msg__PoseWithCovarianceStamped *)msg_id;
    initialpose_msg = *msg;

    Euler angles = QuaternionToEuler(initialpose_msg.pose.pose.orientation.x, 
                                    initialpose_msg.pose.pose.orientation.y, 
                                    initialpose_msg.pose.pose.orientation.z, 
                                    initialpose_msg.pose.pose.orientation.w);

    Robot.Set_Theta(angles.yaw);
    Robot.Set_X_pos(initialpose_msg.pose.pose.position.x);
    Robot.Set_Y_pos(initialpose_msg.pose.pose.position.y);
}