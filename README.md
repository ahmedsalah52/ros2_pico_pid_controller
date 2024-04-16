# ros2 node for raspberry pico microcontroller
### Installation
- follow the example: https://github.com/micro-ROS/micro_ros_raspberrypi_pico_sdk/tree/iron

#### Features:
- 2 Ticks counters for 2 wheels' encoders with external interrupts.
- Subscribes to the cmd_vel topic and controlles the Wheels velocity to the set point using PID controller.
- Subscribes to the init_position topic and resets the Wheel Odometry to this position.
- Calculates the Wheel Odomentry and Publishes the message to the Odom topic.
- Comunicates with the IMU GY521 sensor and publishes the IMU message.
