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

#include "./Controller.hpp"
#include "./MovementFeedback.hpp"

#define SERIAL_PATH "/dev/serial/by-path/pci-0000:00:14.0-usb-0:4:1.2"
#define MSG_ID 10
#define DEBUG_MSG_ID 15
#define GAIN_MSG_ID 20

#define CONTROLLER_TX_ID 0
#define FEEDBACK_RX_ID 1

SerialDev *dev = new LinuxHardwareSerial(SERIAL_PATH, B115200);
SerialBridge serial(dev, 1024);

Gesture gesture_msg;
DebugMessage debug_msg;
ThrowGain gain_msg;

Controller controller_msg;
// MovementFeedback movement_feedback_msg;

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
  // serial.add_frame(MSG_ID, &gesture_msg);
  // serial.add_frame(DEBUG_MSG_ID, &debug_msg);
  // serial.add_frame(GAIN_MSG_ID, &gain_msg);
  serial.add_frame(CONTROLLER_TX_ID, &controller_msg);

  controller_msg.data.movement.x = 0.0;
  controller_msg.data.movement.y = 0.0;
  controller_msg.data.movement.z = 0.8;
  controller_msg.data.all_reload = true;
  controller_msg.data.shooter.num = 11;
  controller_msg.data.shooter.power = 0.1234;
  controller_msg.data.shooter.action = 2;


  //  subscriber
  // ros::Subscriber user_action_sub_ = nh.subscribe("/essentials_detection/user_action", 10, callbackUserAction);
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

    //  SEND DEBUG
    serial.write(CONTROLLER_TX_ID);

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}