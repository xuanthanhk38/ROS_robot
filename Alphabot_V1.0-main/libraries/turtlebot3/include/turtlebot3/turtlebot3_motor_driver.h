#ifndef TURTLEBOT3_MOTOR_DRIVER_H_
#define TURTLEBOT3_MOTOR_DRIVER_H_

#include "Arduino.h"
#include <PID_v1.h>

#define ENCL_A 4 //encorder
#define ENCL_B 5 //encorder

#define ENCR_A 2 //encorder
#define ENCR_B 3 //encorder

#define DIR_R 37
#define DIR_L 44

#define pin_PWM_chanel_0 35 //Perpharal B
#define pin_PWM_chanel_1 42

#define Encoder_cpr 250.0
#define pi 3.1416

#define TORQUE_ENABLE                   1       // Value for enabling the torque
#define TORQUE_DISABLE                  0       // Value for disabling the torque

#define LEFT                            0
#define RIGHT                           1

#define DEBUG_SERIAL  SerialUSB

#define minPWM 0
#define maxPWM 15

const float Kp = 0.05, Ki = 0.7, Kd = 0.09, Kb = 0, alphal = 0; //Kb = anti windup, alpha = (0 - 1)

class Turtlebot3MotorDriver
{
 public:
  float velocity_channel_Left = 0;
  float velocity_channel_Right = 0;
  int32_t encr_diff,encl_diff,encr_prev,encl_prev;

  
  Turtlebot3MotorDriver();
  ~Turtlebot3MotorDriver();
  void _init(void);
  bool setTorque(bool onoff);
  bool getTorque();
  void writeVelocity(int64_t left_value, int64_t right_value);
  bool controlMotor(const float wheel_radius, const float wheel_separation, float* value, int32_t pos_left, int32_t pos_right);
  void startTimer(Tc *tc, uint32_t channel, IRQn_Type irq, uint32_t frequency);
  bool PID_Channel_Left(float desiredValue, float currentValue); //1
  bool PID_Channel_Right(float desiredValue, float currentValue); //0
 private:
  uint32_t baudrate_;
  float  protocol_version_;
  uint8_t left_wheel_id_;
  uint8_t right_wheel_id_;
  bool torque_;

  void setupPWM_chanel_left(int64_t left_value);
  void setupPWM_chanel_right(int64_t right_value);
  
  
};

#endif // TURTLEBOT3_MOTOR_DRIVER_H_
