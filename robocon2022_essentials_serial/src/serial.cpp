#include <ros/ros.h>
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"

#include <iostream>
#include <deque>
#include <math.h>

//	Message
#include <geometry_msgs/Point32.h>
#include <robocon2022_essentials_msgs/UserAction.h>

//  SerialBridge
#include <SerialBridge.hpp>
#include <LinuxHardwareSerial.hpp>
#include "./Gesture.h"

#define SERIAL_PATH "/dev/serial/by-path/pci-0000:00:14.0-usb-0:4:1.2"
#define MSG_ID 10

SerialDev *dev = new LinuxHardwareSerial(SERIAL_PATH, B9600);
SerialBridge serial(dev, 1024);

Gesture gesture_msg;

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

  //  SerialBridge
  serial.add_frame(MSG_ID, &gesture_msg);

  //  subscriber
  ros::Subscriber user_action_sub_ = nh.subscribe("/essentials_detection/user_action", 10, callbackUserAction);

  ros::spin();
  return 0;
}