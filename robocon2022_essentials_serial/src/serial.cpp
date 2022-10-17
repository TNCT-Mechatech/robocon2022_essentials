#include <ros/ros.h>
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float32.h"

#include <iostream>
#include <string.h>
#include <deque>
#include <math.h>

//	Message
#include <geometry_msgs/Point32.h>
#include <robocon2022_essentials_msgs/UserAction.h>

//  SerialBridge
#include <SerialBridge.hpp>
#include <LinuxHardwareSerial.hpp>
#include "./Gesture.h"
#include "./DebugMessage.h"
#include "./ThrowGain.h"

#define SERIAL_PATH "/dev/serial/by-path/pci-0000:00:14.0-usb-0:4:1.2"
#define MSG_ID 10
#define DEBUG_MSG_ID 15
#define GAIN_MSG_ID 20

SerialDev *dev = new LinuxHardwareSerial(SERIAL_PATH, B9600);
SerialBridge serial(dev, 1024);

Gesture gesture_msg;
DebugMessage debug_msg;
ThrowGain gain_msg;

void callbackUserAction(const robocon2022_essentials_msgs::UserAction& userAction)
{
  ROS_INFO("callback");
  gesture_msg.data.type = userAction.action_id - 2;
  serial.write(MSG_ID);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "robocon2022_essentials_serial");
  ros::NodeHandle nh;
  ros::Rate loop_rate(50);

  //  SerialBridge
  serial.add_frame(MSG_ID, &gesture_msg);
  serial.add_frame(DEBUG_MSG_ID, &debug_msg);
  serial.add_frame(GAIN_MSG_ID, &gain_msg);

  //  subscriber
  ros::Subscriber user_action_sub_ = nh.subscribe("/essentials_detection/user_action", 10, callbackUserAction);
  //  publisher
  ros::Publisher debug_pub = nh.advertise<std_msgs::String>("/essentials_serial/debug", 1000);
  ros::Publisher gain_pub = nh.advertise<std_msgs::Float32>("/essentials_serial/gain", 1000);

  while(ros::ok())
  {
    if(serial.update() == 0)
    {
      if(debug_msg.was_updated())
      {
        std_msgs::String msg;
        msg.data = std::string(debug_msg.data.str);
        debug_pub.publish(msg);
      }

      if(gain_msg.was_updated())
      {
        std_msgs::Float32 msg;
        msg.data = gain_msg.data.gain;
        gain_pub.publish(msg);
      }
    }

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}