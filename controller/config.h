//pin out

//motor
#define Left_Motor_PWM_A 10
#define Left_Motor_PWM_B 11

#define Right_Motor_PWM_A 12
#define Right_Motor_PWM_B 13


//encoder
#define Left_Encoder_A_Pin 14
#define Left_Encoder_B_Pin 15
#define Right_Encoder_A_Pin 16
#define Right_Encoder_B_Pin 17
#define TicksPerRevolution 2400

// PID
#define PID_dt   10 //ms
#define PID_dt_sec static_cast<double>(PID_dt)/1000 //seconds
#define L_Kp 3.0
#define L_Ki 1.0
#define L_Kd 0.0

#define R_Kp 3.0
#define R_Ki 1.0
#define R_Kd 0.0

#define counter_res 65535  //datasheet
#define output_res 100
#define L_max output_res
#define L_min -output_res
#define R_max output_res
#define R_min -output_res


//robot specs
//dims in meters
#define wheelbase 0.13  
#define wheel_radius 0.03


//odom
#define Odom_dt 100


