//  the code in the static functions is based on the example in https://github.com/raspberrypi/pico-examples/blob/master/i2c/mpu6050_i2c/mpu6050_i2c.c 
// for the MPU6050 communication with the pico board 

#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/i2c.h"

// #define ACCEL_X_error 1.671875
// #define ACCEL_Y_error 0.001953
// #define ACCEL_Z_error 3.625854

// #define GYRO_X_error 209.099237
// #define GYRO_Y_error 0.244275
// #define GYRO_Z_error 453.480916

//linear velocity error:
#define linear_acc_mean_x 0.1244063749909401
#define linear_acc_mean_y 0.025068603456020355
#define linear_acc_mean_z 1.056947946548462

#define linear_acc_var_x 1.2176247764728032e-05
#define linear_acc_var_y 8.920409527490847e-06
#define linear_acc_var_z 2.2910782718099654e-05

//angular velocity error:
#define angular_vel_mean_x -4.47218132019043
#define angular_vel_mean_y 0.683526873588562
#define angular_vel_mean_z 0.6652669906616211

#define angular_vel_var_x 0.00785013847053051
#define angular_vel_var_y 0.007166045717895031
#define angular_vel_var_z 0.009299776516854763




struct imu_raw_data {
    double accel_x;
    double accel_y;
    double accel_z;
    double gyro_x;
    double gyro_y;
    double gyro_z;
};

struct imu_mean_var {
    double accel_x_mean;
    double accel_y_mean;
    double accel_z_mean;
    double accel_x_var;
    double accel_y_var;
    double accel_z_var;

    double gyro_x_mean;
    double gyro_y_mean;
    double gyro_z_mean;
    double gyro_x_var;
    double gyro_y_var;
    double gyro_z_var;
};


class IMU_MPU6050 {
    public:
        IMU_MPU6050();
        IMU_MPU6050(i2c_inst_t * i2c_port, char sda_pin, char scl_pin, char add_pin, char add_pin_value);
        
        void update_Odom(long double now_sec);
        //void print_Odom();
        imu_mean_var calibrate();
        imu_raw_data get_readings();
        

    private:
        void set_address(char add_pin_value);
        i2c_inst_t *i2c_port;
        char sda_pin;
        char scl_pin;
        char add_pin;
        char add_pin_value;
        char addr;
        int16_t accel_readings[3];
        int16_t gyro_readings[3];
        //positions
        long double X_pos = 0;
        long double Y_pos = 0;
        long double Z_pos = 0;
        //velocities
        long double X_vel = 0;
        long double Y_vel = 0;
        long double Z_vel = 0;  

        //angles
        long double yaw = 0;
        long double pitch = 0;
        long double roll = 0;
        
        //angels velocities
        long double yaw_vel = 0;
        long double pitch_vel = 0;
        long double roll_vel = 0;

        long double prev_time = 0;

        void mpu6050_read_raw(); 
        void mpu6050_reset();
        // constants
        static constexpr float GYRO_SCALE = 131.0; // Scale for gyro readings to get degrees per second
        static constexpr float ACCEL_SCALE = 16384.0; // Scale for accel readings to get g's (9.81 m/s^2)
        static constexpr float PI = 3.14159265358979323846;
        static constexpr float DEG_TO_RAD = PI / 180.0;

};