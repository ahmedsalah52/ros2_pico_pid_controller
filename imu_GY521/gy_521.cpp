#include "gy_521.h"
#include <cmath>


IMU_MPU6050::IMU_MPU6050()
{
    // Default constructor
}
IMU_MPU6050::IMU_MPU6050(i2c_inst_t * i2c_port, char sda_pin, char scl_pin, char add_pin, char add_pin_value)
: 
i2c_port(i2c_port), 
sda_pin(sda_pin), 
scl_pin(scl_pin), 
add_pin(add_pin), 
add_pin_value(add_pin_value)
{   
    // Constructor
    // set add pin to output and set the value of the address pin
    if (add_pin != -1) {
        gpio_init(this->add_pin);
        gpio_set_dir(this->add_pin, GPIO_OUT);
        gpio_put(this->add_pin, this->add_pin_value);
        this->set_address(add_pin_value);
    }
    else
    {
        this->set_address(0);
    }
    

    i2c_init(this->i2c_port, 400 * 1000);
    gpio_set_function(this->sda_pin, GPIO_FUNC_I2C);
    gpio_set_function(this->scl_pin, GPIO_FUNC_I2C);
    gpio_pull_up(this->sda_pin);
    gpio_pull_up(this->scl_pin);
    // Make the I2C pins available to picotool
    bi_decl(bi_2pins_with_func(this->sda_pin, this->scl_pin, GPIO_FUNC_I2C));

    this->mpu6050_reset();
}

void IMU_MPU6050::set_address(char add_pin_value) {
    this->addr= 0x68 + add_pin_value; // 0x68 if default I2C address, 0x69 if alternative I2C address if add_pin_value = 1
}

void IMU_MPU6050::mpu6050_reset() {
    // Two byte reset. First byte register, second byte data
    // There are a load more options to set up the device in different ways that could be added here
    //uint8_t buf[] = {0x6B, 0x80};
    uint8_t buf[] = {0x6b, 0b00000000};
    i2c_write_blocking(this->i2c_port, this->addr, buf, 2, false);

}

void IMU_MPU6050::mpu6050_read_raw() {
    // For this particular device, we send the device the register we want to read
    // first, then subsequently read from the device. The register is auto incrementing
    // so we don't need to keep sending the register we want, just the first.

    uint8_t buffer[6];

    // Start reading acceleration registers from register 0x3B for 6 bytes
    uint8_t val = 0x3B;
    i2c_write_blocking(this->i2c_port, this->addr, &val, 1, true); // true to keep master control of bus
    i2c_read_blocking(this->i2c_port, this->addr, buffer, 6, false);

    for (int i = 0; i < 3; i++) {
        this->accel_readings[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);
    }

    // Now gyro data from reg 0x43 for 6 bytes
    // The register is auto incrementing on each read
    val = 0x43;
    i2c_write_blocking(this->i2c_port, this->addr, &val, 1, true);
    i2c_read_blocking(this->i2c_port, this->addr, buffer, 6, false);  // False - finished with bus

    for (int i = 0; i < 3; i++) {
        this->gyro_readings[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);;
    }

}
imu_mean_var IMU_MPU6050::calibrate() {
    int iterations = 2000;
    float AccErrorX_mean=0, AccErrorY_mean=0, AccErrorZ_mean=0, GyroErrorX_mean=0, GyroErrorY_mean=0, GyroErrorZ_mean=0;
    float AccErrorX_var=0, AccErrorY_var=0, AccErrorZ_var=0, GyroErrorX_var=0, GyroErrorY_var=0, GyroErrorZ_var=0;
    float acc_x_readings[iterations], acc_y_readings[iterations], acc_z_readings[iterations];
    float gyro_x_readings[iterations], gyro_y_readings[iterations], gyro_z_readings[iterations];
    
    for (int i = 0; i < iterations; i++) {
        
        this->mpu6050_read_raw();

        AccErrorX_mean += (float)this->accel_readings[0] / ACCEL_SCALE; //((atan((AccY) / sqrt(pow((AccX), 2) + pow((AccZ), 2))) * 180 / PI));
        AccErrorY_mean += (float)this->accel_readings[1] / ACCEL_SCALE; //((atan(-1 * (AccX) / sqrt(pow((AccY), 2) + pow((AccZ), 2))) * 180 / PI));
        AccErrorZ_mean += (float)this->accel_readings[2] / ACCEL_SCALE;

        GyroErrorX_mean += ((float)this->gyro_readings[0] / GYRO_SCALE);
        GyroErrorY_mean += ((float)this->gyro_readings[1] / GYRO_SCALE);
        GyroErrorZ_mean += ((float)this->gyro_readings[2] / GYRO_SCALE);

        acc_x_readings[i] = (float)this->accel_readings[0] / ACCEL_SCALE;
        acc_y_readings[i] = (float)this->accel_readings[1] / ACCEL_SCALE;
        acc_z_readings[i] = (float)this->accel_readings[2] / ACCEL_SCALE;

        gyro_x_readings[i] = ((float)this->gyro_readings[0] / GYRO_SCALE);
        gyro_y_readings[i] = ((float)this->gyro_readings[1] / GYRO_SCALE);
        gyro_z_readings[i] = ((float)this->gyro_readings[2] / GYRO_SCALE);
    }
    // Calculate the mean
    AccErrorX_mean  /= iterations;
    AccErrorY_mean  /= iterations;
    AccErrorZ_mean  /= iterations;
    GyroErrorX_mean /= iterations;
    GyroErrorY_mean /= iterations;
    GyroErrorZ_mean /= iterations;


    for(int i = 0; i < iterations; i++)
    {
        AccErrorX_var += pow(acc_x_readings[i] - AccErrorX_mean, 2);
        AccErrorY_var += pow(acc_y_readings[i] - AccErrorY_mean, 2);
        AccErrorZ_var += pow(acc_z_readings[i] - AccErrorZ_mean, 2);

        GyroErrorX_var += pow(gyro_x_readings[i] - GyroErrorX_mean, 2);
        GyroErrorY_var += pow(gyro_y_readings[i] - GyroErrorY_mean, 2);
        GyroErrorZ_var += pow(gyro_z_readings[i] - GyroErrorZ_mean, 2);
    }

    AccErrorX_var  /= iterations;
    AccErrorY_var  /= iterations;
    AccErrorZ_var  /= iterations;
    GyroErrorX_var /= iterations;
    GyroErrorY_var /= iterations;
    GyroErrorZ_var /= iterations;

    
    

    imu_mean_var errors;
    errors.accel_x_mean = AccErrorX_mean;
    errors.accel_y_mean = AccErrorY_mean;
    errors.accel_z_mean = AccErrorZ_mean;
    errors.gyro_x_mean  = GyroErrorX_mean;
    errors.gyro_y_mean  = GyroErrorY_mean;
    errors.gyro_z_mean  = GyroErrorZ_mean;

    errors.accel_x_var = AccErrorX_var;
    errors.accel_y_var = AccErrorY_var;
    errors.accel_z_var = AccErrorZ_var;
    errors.gyro_x_var  = GyroErrorX_var;
    errors.gyro_y_var  = GyroErrorY_var;
    errors.gyro_z_var  = GyroErrorZ_var;

    return errors;
    
}
imu_raw_data IMU_MPU6050::get_readings()
{
    this->mpu6050_read_raw();
    imu_raw_data ret;
    ret.accel_x = ((float)this->accel_readings[0] / ACCEL_SCALE) - linear_acc_mean_x; 
    ret.accel_y = ((float)this->accel_readings[1] / ACCEL_SCALE) - linear_acc_mean_y; 
    ret.accel_z = ((float)this->accel_readings[2] / ACCEL_SCALE) - linear_acc_mean_z;

    ret.gyro_x  = (((float)this->gyro_readings[0] / GYRO_SCALE) - angular_vel_mean_x) * this->DEG_TO_RAD;
    ret.gyro_y  = (((float)this->gyro_readings[1] / GYRO_SCALE) - angular_vel_mean_y) * this->DEG_TO_RAD;
    ret.gyro_z  = (((float)this->gyro_readings[2] / GYRO_SCALE) - angular_vel_mean_z) * this->DEG_TO_RAD;

    return ret;
}


void IMU_MPU6050::update_Odom(long double now_sec)
{
    this->mpu6050_read_raw();
    // TODO: Implement the rest of this
    // Convert accelerometer readings from raw to m/s^2, assuming +/- 2g range and 16-bit ADC
    long double dt = (this->prev_time == 0) ? 0.0 : (now_sec - this->prev_time); 
    this->prev_time = now_sec;

    float accel_x_mps2 = (float)this->accel_readings[0] / ACCEL_SCALE * 9.81;
    float accel_y_mps2 = (float)this->accel_readings[1] / ACCEL_SCALE * 9.81;
    float accel_z_mps2 = (float)this->accel_readings[2] / ACCEL_SCALE * 9.81;

    // Convert gyroscope readings from raw to degrees per second
    float gyro_x_dps = (float)this->gyro_readings[0] / GYRO_SCALE;
    float gyro_y_dps = (float)this->gyro_readings[1] / GYRO_SCALE;
    float gyro_z_dps = (float)this->gyro_readings[2] / GYRO_SCALE;

    // Convert degrees per second to radians per second
    float gyro_x_rps = gyro_x_dps * DEG_TO_RAD;
    float gyro_y_rps = gyro_y_dps * DEG_TO_RAD;
    float gyro_z_rps = gyro_z_dps * DEG_TO_RAD;

    //integration of accel and gyro readings

    // calculation of velocity
    this->X_vel = this->X_vel + accel_x_mps2 * dt;
    this->Y_vel = this->Y_vel + accel_y_mps2 * dt;
    this->Z_vel = this->Z_vel + accel_z_mps2 * dt;
    
    // calculation of position
    this->X_pos = this->X_pos + this->X_vel * dt;
    this->Y_pos = this->Y_pos + this->Y_vel * dt;
    this->Z_pos = this->Z_pos + this->Z_vel * dt;

    // calculation of angle
    this->yaw   = this->yaw   + gyro_x_rps * dt;
    this->pitch = this->pitch + gyro_y_rps * dt;
    this->roll  = this->roll  + gyro_z_rps * dt;
}

