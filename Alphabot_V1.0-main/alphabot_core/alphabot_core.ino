

#include "turtlebot3_core_config.h"

#define DEBUG

/*******************************************************************************
* Setup function
*******************************************************************************/
void setup()
{
  DEBUG_SERIAL.begin(57600);

  // Initialize ROS node handle, advertise and subscribe the topics
  nh.initNode();
  //nh.getHardware()->setBaud(115200);

  nh.subscribe(cmd_vel_sub);
  nh.subscribe(motor_power_sub);
  nh.subscribe(reset_sub);

  nh.advertise(sensor_state_pub);
  nh.advertise(version_info_pub);
  nh.advertise(imu_pub);
  nh.advertise(odom_pub);
  nh.advertise(joint_states_pub);
  //nh.advertise(mag_pub);

  tf_broadcaster.init(nh);

  // PWM_init()......
  _pwm_init();

  // Setting for IMU
  sensors.init();
  //sensors.initIMU();

  // Init diagnosis
  diagnosis.init();

  motor_driver._init();

  // Setting for SLAM and navigation (odometry, joint states, TF)
  initOdom();

  initJointStates();

  prev_update_time = millis();

  pinMode(LED_WORKING_CHECK, OUTPUT);

  setup_end = true;
}

/*******************************************************************************
* Loop function
*******************************************************************************/
void loop()
{
  uint32_t t = millis();
  updateTime();
  updateVariable(nh.connected());
  updateTFPrefix(nh.connected());

  if (t - tTime[0] >= (1000 / CONTROL_MOTOR_SPEED_FREQUENCY))
  {
    updateGoalVelocity();
    if ((t - tTime[6]) > CONTROL_MOTOR_TIMEOUT)
    {
      motor_driver.controlMotor(WHEEL_RADIUS, WHEEL_SEPARATION, zero_velocity, sensor_state_msg.left_encoder, sensor_state_msg.right_encoder);
    }
    else
    {
      motor_driver.controlMotor(WHEEL_RADIUS, WHEEL_SEPARATION, goal_velocity, sensor_state_msg.left_encoder, sensor_state_msg.right_encoder);
    }
    //DEBUG_SERIAL.println(t - tTime[0]);
    tTime[0] = t;
  }

  if ((t - tTime[2]) >= (1000 / DRIVE_INFORMATION_PUBLISH_FREQUENCY))
  {
    publishSensorStateMsg(); // delay
    publishDriveInformation();
    tTime[2] = t;
  }

  if ((t - tTime[3]) >= (1000 / IMU_PUBLISH_FREQUENCY))
  {
    publishImuMsg(); // delay
    //publishMagMsg();
    tTime[3] = t;
  }

  if ((t - tTime[4]) >= (1000 / VERSION_INFORMATION_PUBLISH_FREQUENCY))
  {
    publishVersionInfoMsg();
    tTime[4] = t;
  }

#ifdef DEBUG
  if ((t - tTime[5]) >= (1000 / DEBUG_LOG_FREQUENCY))
  {
    sendDebuglog();
    tTime[5] = t;
  }
#endif

  // Send log message after ROS connection
  sendLogMsg();

  /*// Receive data from RC100
  bool clicked_state = controllers.getRCdata(goal_velocity_from_rc100);
  if (clicked_state == true)
    tTime[6] = millis();
  */
  // Update the IMU unit
  sensors.updateIMU();

  // TODO
  // Update sonar data
  // sensors.updateSonar(t);

  // Start Gyro Calibration after ROS connection
  updateGyroCali(nh.connected());

  // Call all the callbacks waiting to be called at that point in time
  nh.spinOnce();

  // Wait the serial link time to process
  waitForSerialLink(nh.connected());
}

/*******************************************************************************
* Callback function for cmd_vel msg
*******************************************************************************/
void commandVelocityCallback(const geometry_msgs::Twist &cmd_vel_msg)
{
  goal_velocity_from_cmd[LINEAR] = cmd_vel_msg.linear.x;
  goal_velocity_from_cmd[ANGULAR] = cmd_vel_msg.angular.z;

  goal_velocity_from_cmd[LINEAR] = constrain(goal_velocity_from_cmd[LINEAR], MIN_LINEAR_VELOCITY, MAX_LINEAR_VELOCITY);
  goal_velocity_from_cmd[ANGULAR] = constrain(goal_velocity_from_cmd[ANGULAR], MIN_ANGULAR_VELOCITY, MAX_ANGULAR_VELOCITY);
  tTime[6] = millis();
}

/*******************************************************************************
* Callback function for motor_power msg
*******************************************************************************/
void motorPowerCallback(const std_msgs::Bool &power_msg)
{
  bool dxl_power = power_msg.data;

  motor_driver.setTorque(dxl_power);
}

/*******************************************************************************
* Callback function for reset msg
*******************************************************************************/
void resetCallback(const std_msgs::Empty &reset_msg)
{
  char log_msg[50];

  (void)(reset_msg);

  sprintf(log_msg, "Start Calibration of Gyro");
  nh.loginfo(log_msg);

  //sensors.calibrationGyro();

  sprintf(log_msg, "Calibration End");
  nh.loginfo(log_msg);

  initOdom();

  sprintf(log_msg, "Reset Odometry");
  nh.loginfo(log_msg);
}

/*******************************************************************************
* Publish msgs (IMU data: angular velocity, linear acceleration, orientation)
*******************************************************************************/
void publishImuMsg(void)
{
  imu_msg = sensors.getIMU();

  imu_msg.header.stamp = rosNow();
  imu_msg.header.frame_id = imu_frame_id;

  imu_pub.publish(&imu_msg);
}

/*******************************************************************************
* Publish msgs (Magnetic data)
*******************************************************************************
void publishMagMsg(void)
{
  mag_msg = sensors.getMag();

  mag_msg.header.stamp    = rosNow();
  mag_msg.header.frame_id = mag_frame_id;

  mag_pub.publish(&mag_msg);
}
*/

/*******************************************************************************
* Publish msgs (sensor_state: bumpers, cliffs, buttons, encoders, battery)
*******************************************************************************/
void publishSensorStateMsg(void)
{
  bool dxl_comm_result = false;

  sensor_state_msg.header.stamp = rosNow();

  updateMotorInfo(sensor_state_msg.left_encoder, sensor_state_msg.right_encoder);

  //sensor_state_msg.illumination = sensors.getIlluminationData();

  sensor_state_msg.torque = motor_driver.getTorque();

  sensor_state_pub.publish(&sensor_state_msg);
}

/*******************************************************************************
* Publish msgs (version info)
*******************************************************************************/
void publishVersionInfoMsg(void)
{
  version_info_msg.hardware = "0.0.0";
  version_info_msg.software = "0.0.0";
  version_info_msg.firmware = FIRMWARE_VER;

  version_info_pub.publish(&version_info_msg);
}

/*******************************************************************************
* Publish msgs (odometry, joint states, tf)
*******************************************************************************/
void publishDriveInformation(void)
{
  unsigned long time_now = millis();
  unsigned long step_time = time_now - prev_update_time;

  prev_update_time = time_now;
  ros::Time stamp_now = rosNow();

  // calculate odometry
  calcOdometry((double)(step_time * 0.001));

  // odometry
  updateOdometry();
  odom.header.stamp = stamp_now;
  odom_pub.publish(&odom);

  // odometry tf
  updateTF(odom_tf);
  odom_tf.header.stamp = stamp_now;
  tf_broadcaster.sendTransform(odom_tf);

  // joint states
  updateJointStates();
  joint_states.header.stamp = stamp_now;
  joint_states_pub.publish(&joint_states);
}

/*******************************************************************************
* Update TF Prefix
*******************************************************************************/
void updateTFPrefix(bool isConnected)
{
  static bool isChecked = false;
  char log_msg[50];

  if (isConnected)
  {
    if (isChecked == false)
    {
      nh.getParam("~tf_prefix", &get_tf_prefix);

      if (!strcmp(get_tf_prefix, ""))
      {
        sprintf(odom_header_frame_id, "odom");
        sprintf(odom_child_frame_id, "base_footprint");

        sprintf(imu_frame_id, "imu_link");
        sprintf(mag_frame_id, "mag_link");
        sprintf(joint_state_header_frame_id, "base_link");
      }
      else
      {
        strcpy(odom_header_frame_id, get_tf_prefix);
        strcpy(odom_child_frame_id, get_tf_prefix);

        strcpy(imu_frame_id, get_tf_prefix);
        strcpy(mag_frame_id, get_tf_prefix);
        strcpy(joint_state_header_frame_id, get_tf_prefix);

        strcat(odom_header_frame_id, "/odom");
        strcat(odom_child_frame_id, "/base_footprint");

        strcat(imu_frame_id, "/imu_link");
        strcat(mag_frame_id, "/mag_link");
        strcat(joint_state_header_frame_id, "/base_link");
      }

      sprintf(log_msg, "Setup TF on Odometry [%s]", odom_header_frame_id);
      nh.loginfo(log_msg);

      sprintf(log_msg, "Setup TF on IMU [%s]", imu_frame_id);
      nh.loginfo(log_msg);

      sprintf(log_msg, "Setup TF on MagneticField [%s]", mag_frame_id);
      nh.loginfo(log_msg);

      sprintf(log_msg, "Setup TF on JointState [%s]", joint_state_header_frame_id);
      nh.loginfo(log_msg);

      isChecked = true;
    }
  }
  else
  {
    isChecked = false;
  }
}

/*******************************************************************************
* Update the odometry
*******************************************************************************/
void updateOdometry(void)
{
  odom.header.frame_id = odom_header_frame_id;
  odom.child_frame_id = odom_child_frame_id;

  odom.pose.pose.position.x = odom_pose[0];
  odom.pose.pose.position.y = odom_pose[1];
  odom.pose.pose.position.z = 0;
  odom.pose.pose.orientation = tf::createQuaternionFromYaw(odom_pose[2]);

  odom.twist.twist.linear.x = odom_vel[0];
  odom.twist.twist.angular.z = odom_vel[2];
}

/*******************************************************************************
* Update the joint states
*******************************************************************************/
void updateJointStates(void)
{
  static float joint_states_pos[WHEEL_NUM] = {0.0, 0.0};
  static float joint_states_vel[WHEEL_NUM] = {0.0, 0.0};
  //static float joint_states_eff[WHEEL_NUM] = {0.0, 0.0};

  joint_states_pos[LEFT] = last_rad[LEFT];
  joint_states_pos[RIGHT] = last_rad[RIGHT];

  joint_states_vel[LEFT] = last_velocity[LEFT];
  joint_states_vel[RIGHT] = last_velocity[RIGHT];

  joint_states.position = joint_states_pos;
  joint_states.velocity = joint_states_vel;
}

/*******************************************************************************
* CalcUpdateulate the TF
*******************************************************************************/
void updateTF(geometry_msgs::TransformStamped &odom_tf)
{
  odom_tf.header = odom.header;
  odom_tf.child_frame_id = odom.child_frame_id;
  odom_tf.transform.translation.x = odom.pose.pose.position.x;
  odom_tf.transform.translation.y = odom.pose.pose.position.y;
  odom_tf.transform.translation.z = odom.pose.pose.position.z;
  odom_tf.transform.rotation = odom.pose.pose.orientation;
}

/*******************************************************************************
* Update motor information
*******************************************************************************/
void updateMotorInfo(int32_t left_tick, int32_t right_tick)
{
  int32_t current_tick = 0;
  static int32_t last_tick[WHEEL_NUM] = {0, 0};

  if (init_encoder)
  {
    for (int index = 0; index < WHEEL_NUM; index++)
    {
      last_diff_tick[index] = 0;
      last_tick[index] = 0;
      last_rad[index] = 0.0;

      last_velocity[index] = 0.0;
    }

    last_tick[LEFT] = left_tick;
    last_tick[RIGHT] = right_tick;

    init_encoder = false;
    return;
  }

  current_tick = left_tick;

  last_diff_tick[LEFT] = current_tick - last_tick[LEFT];
  last_tick[LEFT] = current_tick;
  last_rad[LEFT] += TICK2RAD * (double)last_diff_tick[LEFT];

  current_tick = right_tick;

  last_diff_tick[RIGHT] = current_tick - last_tick[RIGHT];
  last_tick[RIGHT] = current_tick;
  last_rad[RIGHT] += TICK2RAD * (double)last_diff_tick[RIGHT];
}

/*******************************************************************************
* Calculate the odometry
*******************************************************************************/
bool calcOdometry(double diff_time)
{
  float *orientation;
  double wheel_l, wheel_r; // rotation value of wheel [rad]
  double delta_s, theta, delta_theta;
  static double last_theta = 0.0;
  double v, w; // v = translational velocity [m/s], w = rotational velocity [rad/s]
  double step_time;

  wheel_l = wheel_r = 0.0;
  delta_s = delta_theta = theta = 0.0;
  v = w = 0.0;
  step_time = 0.0;

  step_time = diff_time;

  if (step_time == 0)
    return false;

  wheel_l = TICK2RAD * (double)last_diff_tick[LEFT];
  wheel_r = TICK2RAD * (double)last_diff_tick[RIGHT];

  if (isnan(wheel_l))
    wheel_l = 0.0;

  if (isnan(wheel_r))
    wheel_r = 0.0;

  delta_s = WHEEL_RADIUS * (wheel_r + wheel_l) / 2.0;
  // theta = WHEEL_RADIUS * (wheel_r - wheel_l) / WHEEL_SEPARATION;
  orientation = sensors.getOrientation();
  theta = atan2f(orientation[1] * orientation[2] + orientation[0] * orientation[3],
                 0.5f - orientation[2] * orientation[2] - orientation[3] * orientation[3]);

  delta_theta = theta - last_theta;

  // compute odometric pose
  odom_pose[0] += delta_s * cos(odom_pose[2] + (delta_theta / 2.0));
  odom_pose[1] += delta_s * sin(odom_pose[2] + (delta_theta / 2.0));
  odom_pose[2] += delta_theta;

  // compute odometric instantaneouse velocity

  v = delta_s / step_time;
  w = delta_theta / step_time;

  odom_vel[0] = v;
  odom_vel[1] = 0.0;
  odom_vel[2] = w;

  last_velocity[LEFT] = wheel_l / step_time;
  last_velocity[RIGHT] = wheel_r / step_time;
  last_theta = theta;

  return true;
}
/*******************************************************************************
* Update variable (initialization)
*******************************************************************************/
void updateVariable(bool isConnected)
{
  static bool variable_flag = false;

  if (isConnected)
  {
    if (variable_flag == false)
    {
      sensors.initIMU();
      initOdom();

      variable_flag = true;
    }
  }
  else
  {
    variable_flag = false;
  }
}

/*******************************************************************************
* Wait for Serial Link
*******************************************************************************/
void waitForSerialLink(bool isConnected)
{
  static bool wait_flag = false;

  if (isConnected)
  {
    if (wait_flag == false)
    {
      delay(10);

      wait_flag = true;
    }
  }
  else
  {
    wait_flag = false;
  }
}

/*******************************************************************************
* Update the base time for interpolation
*******************************************************************************/
void updateTime()
{
  current_offset = millis();
  current_time = nh.now();
}

/*******************************************************************************
* ros::Time::now() implementation
*******************************************************************************/
ros::Time rosNow()
{
  return nh.now();
}

/*******************************************************************************
* Time Interpolation function (deprecated)
*******************************************************************************/
ros::Time addMicros(ros::Time &t, uint32_t _micros)
{
  uint32_t sec, nsec;

  sec = _micros / 1000 + t.sec;
  nsec = _micros % 1000000000 + t.nsec;

  return ros::Time(sec, nsec);
}

/*******************************************************************************
* Start Gyro Calibration
*******************************************************************************/
void updateGyroCali(bool isConnected)
{
  static bool isEnded = false;
  char log_msg[50];

  (void)(isConnected);

  if (nh.connected())
  {
    if (isEnded == false)
    {
      sprintf(log_msg, "Start Calibration of Gyro");
      nh.loginfo(log_msg);

      //sensors.calibrationGyro();

      sprintf(log_msg, "Calibration End");
      nh.loginfo(log_msg);

      isEnded = true;
    }
  }
  else
  {
    isEnded = false;
  }
}

/*******************************************************************************
* Send log message
*******************************************************************************/
void sendLogMsg(void)
{
  static bool log_flag = false;
  char log_msg[100];

  String name = NAME;
  String firmware_version = FIRMWARE_VER;
  String bringup_log = "This core(v" + firmware_version + ") is compatible with TB3 " + name;

  const char *init_log_data = bringup_log.c_str();

  if (nh.connected())
  {
    if (log_flag == false)
    {
      sprintf(log_msg, "--------------------------");
      nh.loginfo(log_msg);

      sprintf(log_msg, "Connected to OpenCR board!");
      nh.loginfo(log_msg);

      sprintf(log_msg, init_log_data);
      nh.loginfo(log_msg);

      sprintf(log_msg, "--------------------------");
      nh.loginfo(log_msg);

      log_flag = true;
    }
  }
  else
  {
    log_flag = false;
  }
}

/*******************************************************************************
* Initialization odometry data
*******************************************************************************/
void initOdom(void)
{
  init_encoder = true;

  for (int index = 0; index < 3; index++)
  {
    odom_pose[index] = 0.0;
    odom_vel[index] = 0.0;
  }

  odom.pose.pose.position.x = 0.0;
  odom.pose.pose.position.y = 0.0;
  odom.pose.pose.position.z = 0.0;

  odom.pose.pose.orientation.x = 0.0;
  odom.pose.pose.orientation.y = 0.0;
  odom.pose.pose.orientation.z = 0.0;
  odom.pose.pose.orientation.w = 0.0;

  odom.twist.twist.linear.x = 0.0;
  odom.twist.twist.angular.z = 0.0;
}

/*******************************************************************************
* Initialization joint states data
*******************************************************************************/
void initJointStates(void)
{
  static char *joint_states_name[] = {(char *)"wheel_left_joint", (char *)"wheel_right_joint"};

  joint_states.header.frame_id = joint_state_header_frame_id;
  joint_states.name = joint_states_name;

  joint_states.name_length = WHEEL_NUM;
  joint_states.position_length = WHEEL_NUM;
  joint_states.velocity_length = WHEEL_NUM;
  joint_states.effort_length = WHEEL_NUM;
}

/*******************************************************************************
* Update Goal Velocity
*******************************************************************************/
void updateGoalVelocity(void)
{
  goal_velocity[LINEAR] = goal_velocity_from_button[LINEAR] + goal_velocity_from_cmd[LINEAR] + goal_velocity_from_rc100[LINEAR];
  goal_velocity[ANGULAR] = goal_velocity_from_button[ANGULAR] + goal_velocity_from_cmd[ANGULAR] + goal_velocity_from_rc100[ANGULAR];
}

/*******************************************************************************
* Send Debug data
*******************************************************************************/
void sendDebuglog(void)
{
  //DEBUG_SERIAL.println("---------------------------------------");
  //DEBUG_SERIAL.println("EXTERNAL SENSORS");
  //DEBUG_SERIAL.println("---------------------------------------");
  //DEBUG_SERIAL.print("Bumper : "); DEBUG_SERIAL.println(sensors.checkPushBumper());
  //DEBUG_SERIAL.print("Cliff : "); DEBUG_SERIAL.println(sensors.getIRsensorData());
  //DEBUG_SERIAL.print("Sonar : "); DEBUG_SERIAL.println(sensors.getSonarData());
  //DEBUG_SERIAL.print("Illumination : "); DEBUG_SERIAL.println(sensors.getIlluminationData());

  //DEBUG_SERIAL.println("---------------------------------------");
  //DEBUG_SERIAL.println("OpenCR SENSORS");
  //DEBUG_SERIAL.println("---------------------------------------");
  //DEBUG_SERIAL.print("Battery : "); DEBUG_SERIAL.println(sensors.checkVoltage());
  //DEBUG_SERIAL.println("Button : " + String(sensors.checkPushButton()));

  float *quat = sensors.getOrientation();

  //DEBUG_SERIAL.println("IMU : ");
  //DEBUG_SERIAL.print("    w : ");
  //DEBUG_SERIAL.println(quat[0]);
  //DEBUG_SERIAL.print("    x : ");
  //DEBUG_SERIAL.println(quat[1]);
  //DEBUG_SERIAL.print("    y : ");
  //DEBUG_SERIAL.println(quat[2]);
  //DEBUG_SERIAL.print("    z : ");
  //DEBUG_SERIAL.println(quat[3]);

  //DEBUG_SERIAL.println("---------------------------------------");
  //DEBUG_SERIAL.println("DYNAMIXELS");
  //DEBUG_SERIAL.println("---------------------------------------");
  //DEBUG_SERIAL.println("Torque : " + String(motor_driver.getTorque()));

  //DEBUG_SERIAL.println("Encoder(left) : " + String(pos_left));
  //DEBUG_SERIAL.println("Encoder(right) : " + String(pos_right));

  //DEBUG_SERIAL.println("---------------------------------------");
  //DEBUG_SERIAL.println("TurtleBot3");
  //DEBUG_SERIAL.println("---------------------------------------");
  //DEBUG_SERIAL.println("Odometry : ");
  //DEBUG_SERIAL.print("         x : ");
  //DEBUG_SERIAL.println(odom_pose[0]);
  //DEBUG_SERIAL.print("         y : ");
  //DEBUG_SERIAL.println(odom_pose[1]);
  //DEBUG_SERIAL.print("     theta : ");
  //DEBUG_SERIAL.println(odom_pose[2]);
}

void _pwm_init()
{
  pinMode(ENCR_A, INPUT);
  pinMode(ENCR_B, INPUT);
  pinMode(ENCL_A, INPUT);
  pinMode(ENCL_B, INPUT);
  attachInterrupt(ENCR_B, Callback_R, RISING);
  attachInterrupt(ENCL_B, Callback_L, RISING);

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
}

void Callback_R() //callback when _B rising
{
  if (!digitalRead(ENCR_A))
  {
    pos_right++;
    sensor_state_msg.right_encoder = pos_right;
  }
  else
  {
    pos_right--;
    sensor_state_msg.right_encoder = pos_right;
  }
}

void Callback_L() //callback when _B rising
{
  if (digitalRead(ENCL_A))
  {
    pos_left++;
    sensor_state_msg.left_encoder = pos_left;
  }
  else
  {
    pos_left--;
    sensor_state_msg.left_encoder = pos_left;
  }
}
void TC3_Handler()
{
  TC_GetStatus(TC1, 0);
  motor_driver.encr_diff = pos_right - motor_driver.encr_prev;
  motor_driver.encl_diff = pos_left - motor_driver.encl_prev;

  motor_driver.encr_prev = pos_right;
  motor_driver.encl_prev = pos_left;

  //DEBUG_SERIAL.print("pos_left");
  //DEBUG_SERIAL.print(pos_left);
  //DEBUG_SERIAL.print("\t");
  //DEBUG_SERIAL.print("post_right");
  //DEBUG_SERIAL.println(pos_right);

  // DEBUG_SERIAL.print("encr:"); DEBUG_SERIAL.println(motor_driver.encr_diff);
  // DEBUG_SERIAL.print("encl:"); DEBUG_SERIAL.println(motor_driver.encl_diff);

  float velocity_now_left = ((motor_driver.encl_diff * 60.0 / 250) / 0.1);
  //encorder_count_channel_1 = 0;

  float velocity_now_right = ((motor_driver.encr_diff * 60.0 / 250) / 0.1);
  //encorder_count_channel_0 = 0;

  //DEBUG_SERIAL.print("vel_r:"); DEBUG_SERIAL.println(velocity_now_right);
  //DEBUG_SERIAL.print("vel_l:"); DEBUG_SERIAL.println(velocity_now_left);

  float temp_left = (((float)motor_driver.encl_diff / 250) * 0.5024) / 0.1;

  float temp_right = (((float)motor_driver.encr_diff / 250) * 0.5024) / 0.1;

  //DEBUG_SERIAL.print("vel_r:"); DEBUG_SERIAL.println(temp_right);

  //DEBUG_SERIAL.print("vel_l:"); DEBUG_SERIAL.println(temp_left);

  motor_driver.PID_Channel_Left(motor_driver.velocity_channel_Left, temp_left);    //left
  motor_driver.PID_Channel_Right(motor_driver.velocity_channel_Right, temp_right); //right
}
