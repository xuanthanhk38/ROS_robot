// C library headers
#include <stdio.h>
#include <string>
#include <cstring>
#include <iostream>
#include <sstream>
// Linux headers
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()
#include <sys/ioctl.h>
#include "ros/ros.h"
#include "ros/time.h"
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Quaternion.h"
#include "sensor_msgs/JointState.h"
#include "sensor_msgs/Imu.h"
#include "tf/transform_broadcaster.h"
#include "turtlebot3_msgs/VersionInfo.h"
#include "turtlebot3_msgs/SensorState.h"

/**********************Define publish/subscribe cycle**********************/

#define FIRMWARE_VER "1.2.6"

#define CONTROL_MOTOR_SPEED_FREQUENCY          30   //hz
#define CONTROL_MOTOR_TIMEOUT                  500  //ms
#define IMU_PUBLISH_FREQUENCY                  200  //hz
#define CMD_VEL_PUBLISH_FREQUENCY              30   //hz
#define DRIVE_INFORMATION_PUBLISH_FREQUENCY    10   //hz
#define VERSION_INFORMATION_PUBLISH_FREQUENCY  1    //hz 
#define DEBUG_LOG_FREQUENCY                    10   //hz 
#define TICK2RAD                               0.0000116296296 //Default tick2rad waffle or waffle pi
#define WHEEL_RADIUS                           0.0725 //Default waffle or waffle pi
#define WHEEL_SEPARATION                       0.292 //Default waffle or waffle pi

/**********************Define publish/subscribe cycle**********************/

/**********************Processing class**********************/
class SerialPort //Serial class
{
    private:        
        termios tty;    
        char buffer [256];   
        int count = 0;
        int num_bytes;      
    public:      
        int serial_port;
        bool initial(const char* portname) //Initial serial port
        {   
            bool in_return = false;
            tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
            tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
            tty.c_cflag &= ~CSIZE; // Clear all bits that set the data size 
            tty.c_cflag |= CS8; // 8 bits per byte (most common)
            tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
            tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

            tty.c_lflag &= ~ICANON;
            tty.c_lflag &= ~ECHO; // Disable echo
            tty.c_lflag &= ~ECHOE; // Disable erasure
            tty.c_lflag &= ~ECHONL; // Disable new-line echo
            tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
            tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
            tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes

            tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
            tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
            cfsetspeed(&tty, B115200);  
            tty.c_cc[VTIME] = 3;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
            tty.c_cc[VMIN] = 0;
            serial_port = open(portname, O_RDWR | O_NOCTTY | O_NDELAY);
            

            tcflush( serial_port, TCIFLUSH );
            if ( tcsetattr ( serial_port, TCSANOW, &tty ) != 0) {
                std::cout << "Error " << errno << " from tcsetattr" << std::endl;
                in_return = false;
            }
            else in_return = true;
            
            return in_return;       
        }               

        bool available()
        {
            int nread = 0;
            ioctl(serial_port, FIONREAD, &nread);
            if (nread > 0) {
                return true;
            }
            else return false;
        }   

        std::string readString()
        {
            memset(&buffer, '\0', sizeof(buffer));
            int num_bytes = read(serial_port, &buffer, sizeof(buffer));
            return std::string(buffer);
        }

        std::string readStringUntilNewline()
        {
            char c[1];
            std::string in_return = "";
            read(serial_port, &c, 1);
            if(c[0] != '\n')
            {
                buffer[count] = c[0];
                count++;
                return in_return;
            }
            else  
            {
                count = 0;
                in_return = std::string(buffer);
                memset(&buffer, '\0', sizeof(buffer));
                return in_return;
            }       
        }

        void Close()
        {
            close(serial_port);
        }

        void Clear()
        {
            ioctl(serial_port, TCFLSH, 2);
        }
};

bool isOnlyDouble(const char* str)
{
    char* endptr = 0;
    strtod(str, &endptr);

    if(*endptr != '\0' || endptr == str)
        return false;
    return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker");
  ros::NodeHandle n;

  /**********************Advertise pub / sub**********************/
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 20);
  ros::Publisher sensor_state_pub = n.advertise<turtlebot3_msgs::SensorState>("sensor_state", 20);
  ros::Publisher joint_state_pub = n.advertise<sensor_msgs::JointState>("joint_states", 20);
  ros::Publisher imu_pub = n.advertise<sensor_msgs::Imu>("imu", 20);
  ros::Publisher version_info_pub = n.advertise<turtlebot3_msgs::VersionInfo>("firmware_version", 20);
  /**********************Advertise pub / sub**********************/

  /**********************ROS time and frequency**********************/
  ros::Time current_time, lastOdomPublishFrequency, lastVersionPublishFrequency, lastImuPublishFrequency, last_time;
  current_time = ros::Time::now();
  lastOdomPublishFrequency = ros::Time::now();
  lastVersionPublishFrequency = ros::Time::now();
  lastImuPublishFrequency = ros::Time::now();
  
  ros::Duration OdomPublishFrequency(1 / DRIVE_INFORMATION_PUBLISH_FREQUENCY);
  ros::Duration VersionPublishFrequency(1 / VERSION_INFORMATION_PUBLISH_FREQUENCY);
  ros::Duration ImuPublishFrequency(1 / IMU_PUBLISH_FREQUENCY);
  /**********************ROS time and frequency**********************/

  tf::TransformBroadcaster odom_broadcaster;

  /**********************Init serial port**********************/
  SerialPort serial;
  serial.initial("/dev/ttyS0");
  serial.Clear();
  /**********************Init serial port**********************/

  /**********************Debug data**********************/
  long encoder_left_tick = 0;
  long encoder_right_tick = 0;

  long previous_encoder_left_tick = 0;
  long previous_encoder_right_tick = 0;

  double last_theta = 0;
  double odom_pose[3];
  double odom_vel[3];
  /**********************Debug data**********************/

  while (ros::ok())
  {
    if(serial.available())
    {   
        std::string data[13];
        std::string in_return = serial.readStringUntilNewline();
        if(in_return.length() > 0)    
        {
            current_time = ros::Time::now();
            //std::cout<<in_return<<std::endl;
            int _count = 0;
            std::string delimiter = "|";
            size_t pos = 0;
            std::string token;
            while ((pos = in_return.find(delimiter)) != std::string::npos) {
                token = in_return.substr(0, pos);
                data[_count] = token;
                _count = _count + 1;
                in_return.erase(0, pos + delimiter.length());
            }
            data[12] = in_return;

            geometry_msgs::Quaternion odom_quat;

            bool correct = true;

            double orienw = 0;
            double orienx = 0;
            double orieny = 0;
            double orienz = 0;

            double gx = 0;
            double gy = 0;
            double gz = 0;

            double ax = 0;
            double ay = 0;
            double az = 0;

            double process_time = std::stod(data[12]);


            try
            {
                orienw = std::stod(data[2]);
                orienx = std::stod(data[3]);
                orieny = std::stod(data[4]);
                orienz = std::stod(data[5]);

                gx = std::stod(data[6]);
                gy = std::stod(data[7]);
                gz = std::stod(data[8]);

                ax = std::stod(data[9]);
                ay = std::stod(data[10]);
                az = std::stod(data[11]);

                encoder_left_tick = std::stoi(data[0]);   
                encoder_right_tick = std::stoi(data[1]);

                process_time = std::stod(data[12]);
            }
            catch(const std::exception& e)
            {
                correct = false;
            }                

            if(correct)
            {
                /**************************DEBUG calculate odometry, ect...**************************/

                long diff_left = encoder_left_tick - previous_encoder_left_tick;
                previous_encoder_left_tick = encoder_left_tick;

                long diff_right = encoder_right_tick - previous_encoder_right_tick;
                previous_encoder_right_tick = encoder_right_tick;

                double wheel_l = 0;
                double wheel_r = 0;

                double delta_s = 0;
                double theta = 0;
                double delta_theta = 0;

                double v = 0;
                double w = 0;

                wheel_l = TICK2RAD * (double)diff_left;
                wheel_r = TICK2RAD * (double)diff_right;

                delta_s = WHEEL_RADIUS * (wheel_r + wheel_l) / 2.0;

                theta = atan2f(orienx * orieny + orienw * orienz,
                 0.5f - orieny * orieny - orienz * orienz);

                delta_theta = theta - last_theta;
                last_theta = theta;

                odom_pose[0] += delta_s * cos(odom_pose[2] + (delta_theta / 2.0));
                odom_pose[1] += delta_s * sin(odom_pose[2] + (delta_theta / 2.0));
                odom_pose[2] += delta_theta;

                v = delta_s / (process_time / 1000000);
                w = delta_theta / (process_time / 1000000);

                odom_vel[0] = v;
                odom_vel[1] = 0.0;
                odom_vel[2] = w;

                /**************************DEBUG calculate odometry, ect...**************************/

                odom_quat.x = orienx;
                odom_quat.y = orieny;
                odom_quat.z = orienz;
                odom_quat.w = orienw;

                geometry_msgs::TransformStamped odom_trans;
                odom_trans.header.stamp = current_time;
                odom_trans.header.frame_id = "odom";
                odom_trans.child_frame_id = "base_footprint";

                odom_trans.transform.translation.x = odom_pose[0];
                odom_trans.transform.translation.y = odom_pose[1];
                odom_trans.transform.translation.z = 0.0;

                odom_trans.transform.rotation.x = odom_quat.x;
                odom_trans.transform.rotation.y = odom_quat.y;
                odom_trans.transform.rotation.z = odom_quat.z;
                odom_trans.transform.rotation.w = odom_quat.w;

                //send the transform
                odom_broadcaster.sendTransform(odom_trans);

                //next, we'll publish the odometry message over ROS
                nav_msgs::Odometry odom;
                odom.header.stamp = current_time;
                odom.header.frame_id = "odom";

                //set the position
                odom.pose.pose.position.x = odom_pose[0];
                odom.pose.pose.position.y = odom_pose[1];
                odom.pose.pose.position.z = 0;
                odom.pose.pose.orientation.x = 0;
                odom.pose.pose.orientation.y = 0;
                odom.pose.pose.orientation.z = odom_quat.z;
                odom.pose.pose.orientation.w = odom_quat.w;

                //set the velocity
                odom.child_frame_id = "base_footprint";
                odom.twist.twist.linear.x = odom_vel[0];
                odom.twist.twist.angular.z = odom_vel[2];

                //publish the message
                odom_pub.publish(odom);

                /*********Publish sensor state*********/
                turtlebot3_msgs::SensorState sensor_state_msg;
                sensor_state_msg.header.stamp = current_time;
                sensor_state_msg.left_encoder = encoder_left_tick;
                sensor_state_msg.right_encoder = encoder_right_tick;
                sensor_state_msg.torque = 0;
                sensor_state_pub.publish(sensor_state_msg);
                /*********Publish sensor state*********/

                /*********Publish joint state*********/
                sensor_msgs::JointState joint_states;
                static std::string joint_states_name[] = {"wheel_left_joint", "wheel_right_joint"};
                joint_states.header.frame_id = "base_link";
                joint_states.name.push_back(joint_states_name[1]);
                joint_states.name.push_back(joint_states_name[0]);
                joint_states.velocity.push_back(0.0);
                joint_states.velocity.push_back(0.0);
                joint_states.position.push_back(0.0);
                joint_states.position.push_back(0.0);
                joint_states.header.stamp = current_time;
                joint_state_pub.publish(joint_states);
                /*********Publish joint state*********/


                turtlebot3_msgs::VersionInfo _version;
                _version.hardware = "0.0.0";
                _version.software = "0.0.0";
                _version.firmware = FIRMWARE_VER;

                version_info_pub.publish(_version);
            }           
        }
    }

    ros::spinOnce();
  }

  serial.Close();
}


