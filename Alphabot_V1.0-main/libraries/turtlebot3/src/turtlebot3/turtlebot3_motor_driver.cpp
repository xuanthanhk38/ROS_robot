#include "../../include/turtlebot3/turtlebot3_motor_driver.h"

// PID Parameters

Turtlebot3MotorDriver::Turtlebot3MotorDriver()
{
}

Turtlebot3MotorDriver::~Turtlebot3MotorDriver()
{
}

double InputLeft, OutputLeft, SetpointLeft;
double InputRight, OutputRight, SetpointRight;

PID LeftPID(&InputLeft, &OutputLeft, &SetpointLeft, 30, 20, 0.05, DIRECT);
PID RightPID(&InputRight, &OutputRight, &SetpointRight, 30 , 20, 0.05, DIRECT);

double speed_req_left = 0; //Desired speed for left wheel in m/s
double speed_act_left = 0; //Actual speed for left wheel in m/s
double speed_cmd_left = 0; //Command speed for left wheel in m/s

double speed_req_right = 0; //Desired speed for right wheel in m/s
double speed_act_right = 0; //Actual speed for right wheel in m/s
double speed_cmd_right = 0; //Command speed for right wheel in m/s

const double max_speed = 0.3; //Max speed in m/s

void Turtlebot3MotorDriver::_init()
{
  DEBUG_SERIAL.begin(57600);

  //chanel 0
  REG_PMC_PCER1 |= PMC_PCER1_PID36;
  REG_PIOC_ABSR |= PIO_ABSR_P3;
  REG_PIOC_PDR |= PIO_PDR_P3;                      //disable from controlling the corres (enable perpheral)
  REG_PWM_CLK = PWM_CLK_PREB(0) | PWM_CLK_DIVB(2); //84 / 42 = 2M
  REG_PWM_CMR0 = PWM_CMR_CALG | PWM_CMR_CPRE_CLKB;

  //chanel 1
  REG_PMC_PCER1 |= PMC_PCER1_PID36;
  REG_PIOA_ABSR |= PIO_ABSR_P19;
  REG_PIOA_PDR |= PIO_PDR_P19;                     //disable from controlling the corres (enable perpheral)
  REG_PWM_CLK = PWM_CLK_PREB(0) | PWM_CLK_DIVB(2); //84 / 42 = 2M
  REG_PWM_CMR1 = PWM_CMR_CALG | PWM_CMR_CPRE_CLKB;

  this->startTimer(TC1, 0, TC3_IRQn, 10);

  pinMode(DIR_R, OUTPUT);
  pinMode(DIR_L, OUTPUT);
}
void Turtlebot3MotorDriver::startTimer(Tc *tc, uint32_t channel, IRQn_Type irq, uint32_t frequency)
{
  pmc_set_writeprotect(false);
  pmc_enable_periph_clk((uint32_t)irq);
  TC_Configure(tc, channel, TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC | TC_CMR_TCCLKS_TIMER_CLOCK3);
  uint32_t rc = VARIANT_MCK / 32 / frequency; //128 because we selected TIMER_CLOCK4 above
  TC_SetRA(tc, channel, rc / 2);              //50% high, 50% low
  TC_SetRC(tc, channel, rc);
  TC_Start(tc, channel);
  tc->TC_CHANNEL[channel].TC_IER = TC_IER_CPCS;
  tc->TC_CHANNEL[channel].TC_IDR = ~TC_IER_CPCS;
  NVIC_EnableIRQ(irq);
}
bool Turtlebot3MotorDriver::setTorque(bool onoff)
{
  torque_ = onoff;
  return true;
}

bool Turtlebot3MotorDriver::getTorque()
{
  return torque_;
}

void Turtlebot3MotorDriver::writeVelocity(int64_t left_value, int64_t right_value)
{
  // chanel 0
  REG_PWM_CPRD0 = 2100; //42M / (2*2100) = 10khz
  REG_PWM_CDTY0 = 2100 * ((100 - left_value) * 1.0 / 100);
  REG_PWM_ENA = PWM_ENA_CHID0;

  REG_PWM_CPRD1 = 2100; //42M / (2*2100) = 10khz
  REG_PWM_CDTY1 = 2100 * ((100 - right_value) * 1.0 / 100);
  REG_PWM_ENA = PWM_ENA_CHID1;
}

void Turtlebot3MotorDriver::setupPWM_chanel_right(int64_t right_value)
{
  REG_PWM_CPRD1 = 2100; //42M / (2*2100) = 10khz
  REG_PWM_CDTY1 = 2100 * ((100 - right_value) * 1.0 / 100);
  REG_PWM_ENA = PWM_ENA_CHID1;
}

void Turtlebot3MotorDriver::setupPWM_chanel_left(int64_t left_value)
{
  REG_PWM_CPRD0 = 2100; //42M / (2*2100) = 10khz
  REG_PWM_CDTY0 = 2100 * ((100 - left_value) * 1.0 / 100);
  REG_PWM_ENA = PWM_ENA_CHID0;
}

bool Turtlebot3MotorDriver::controlMotor(const float wheel_radius, const float wheel_separation, float *value, int32_t pos_left, int32_t pos_right)
{
  float wheel_velocity_cmd[2];

  float lin_vel = value[LEFT];
  float ang_vel = value[RIGHT];




  wheel_velocity_cmd[LEFT] = lin_vel - (ang_vel * wheel_separation / 2);
  wheel_velocity_cmd[RIGHT] = lin_vel + (ang_vel * wheel_separation / 2);


  //DEBUG_SERIAL.println(wheel_velocity_cmd[LEFT]);



  wheel_velocity_cmd[LEFT] = constrain(wheel_velocity_cmd[LEFT], -max_speed, max_speed);
  wheel_velocity_cmd[RIGHT] = constrain(wheel_velocity_cmd[RIGHT], -max_speed, max_speed);

  //wheel_velocity_cmd[LEFT] = (wheel_velocity_cmd[LEFT] / (2 * pi * wheel_radius)) * 60;
  //wheel_velocity_cmd[RIGHT] = (wheel_velocity_cmd[RIGHT] / (2 * pi * wheel_radius)) * 60;

  this->velocity_channel_Left = wheel_velocity_cmd[LEFT];
  this->velocity_channel_Right = wheel_velocity_cmd[RIGHT];

  // DEBUG_SERIAL.print("vel_set_left:");
  // DEBUG_SERIAL.println(velocity_channel_Left);
  // DEBUG_SERIAL.print("vel_set_right:");
  // DEBUG_SERIAL.println(velocity_channel_Right);

  // char tmp[255] = {0};

  // sprintf(tmp, "wheel_velocity_cmd_left %f", wheel_velocity_cmd[LEFT]);
  // DEBUG_SERIAL.println(tmp);
  // sprintf(tmp, "wheel_velocity_cmd_right %f", wheel_velocity_cmd[RIGHT]);
  // DEBUG_SERIAL.println(tmp);


  if (wheel_velocity_cmd[LEFT] < 0)
    digitalWrite(DIR_L, 1);
  else if (wheel_velocity_cmd[LEFT] > 0)
    digitalWrite(DIR_L, 0);

  if (wheel_velocity_cmd[RIGHT] < 0)
    digitalWrite(DIR_R, 0);
  else if (wheel_velocity_cmd[RIGHT] > 0)
    digitalWrite(DIR_R, 1);

  //writeVelocity(abs(speed_cmd_left), abs(speed_cmd_right));
  return true;
}

bool Turtlebot3MotorDriver::PID_Channel_Left(float desiredValue, float currentValue)
{
  /*static float err_p = 0;
  static float u_i_p = 0;
  static float u_d_p = 0;
  static float err_reset = 0;
  static const float sampleTime = 0.1; //100ms
  float err, err_windup;
  float u_p, u_i, u_d;
  int u_out;*/



  
  if (desiredValue < 0)
  {
    desiredValue = -desiredValue;
  }

  //if(desiredValue < 0.11f) desiredValue = 0;


  if (currentValue < 0)
  {
    currentValue = -currentValue;
  }

  /*err = desiredValue - currentValue;

  u_p = Kp * err;

  err_windup = Ki * err + Kb * err_reset;

  u_i = u_i_p + err_windup * sampleTime;

  u_d = Kd * (err - err_p) / sampleTime;
  u_d = (1 - alphal) * u_d_p + alphal * u_d;

  err_p = err;
  u_i_p = u_i;
  u_d_p = u_d;

  u_out = (int)(u_p + u_i + u_d);*/

  InputLeft = currentValue;
  SetpointLeft = desiredValue;
  LeftPID.SetMode(AUTOMATIC);
  LeftPID.SetSampleTime(100);
  LeftPID.Compute();

  if (OutputLeft > maxPWM)
  {
    OutputLeft = maxPWM;
  }
  else if (OutputLeft < minPWM)
  {
    OutputLeft = minPWM;
  }

  if(desiredValue == 0) OutputLeft = 0;

  this->setupPWM_chanel_left(OutputLeft); //left - 1

  // if (u_out <= 0)
  // {
  //   digitalWrite(pin_dir_chanel_0, HIGH);
  // }
  // else
  // {
  //   digitalWrite(pin_dir_chanel_0, LOW);
  // }

  return true;
}

bool Turtlebot3MotorDriver::PID_Channel_Right(float desiredValue, float currentValue)
{
  /*static float err_p = 0;
  static float u_i_p = 0;
  static float u_d_p = 0;
  static float err_reset = 0;
  static const float sampleTime = 0.1; //100ms
  float err, err_windup;
  float u_p, u_i, u_d;
  int u_out;*/

  if (desiredValue < 0)
  {
    desiredValue = -desiredValue;
  }

  //if (desiredValue < 0.11f) desiredValue = 0;


  if (currentValue < 0)
  {
    currentValue = -currentValue;
  }

  /*err = desiredValue - currentValue;

  u_p = Kp * err;

  err_windup = Ki * err + Kb * err_reset;

  u_i = u_i_p + err_windup * sampleTime;

  u_d = Kd * (err - err_p) / sampleTime;
  u_d = (1 - alphal) * u_d_p + alphal * u_d;

  err_p = err;
  u_i_p = u_i;
  u_d_p = u_d;

  u_out = (int)(u_p + u_i + u_d);*/

  InputRight = currentValue;
  SetpointRight = desiredValue;
  RightPID.SetMode(AUTOMATIC);
  RightPID.SetSampleTime(100);
  RightPID.Compute();

  DEBUG_SERIAL.print("PWM Right: ");   DEBUG_SERIAL.println(OutputRight); 
  DEBUG_SERIAL.print("Desire Right: ");   DEBUG_SERIAL.println(desiredValue); 
  DEBUG_SERIAL.print("Current Right: ");   DEBUG_SERIAL.println(currentValue); 

  if (OutputRight > maxPWM)
  {
    OutputRight = maxPWM;
  }
  else if (OutputRight < minPWM)
  {
    OutputRight = minPWM;
  }

  if(desiredValue == 0) OutputRight = 0;

  this->setupPWM_chanel_right(OutputRight); //Right - 1

  // if (u_out <= 0)
  // {
  //   digitalWrite(pin_dir_chanel_0, HIGH);
  // }
  // else
  // {
  //   digitalWrite(pin_dir_chanel_0, LOW);
  // }

  return true;
}
