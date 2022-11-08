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
#include <robocon2022_essentials_msgs/Controller.h>

//  SerialBridge
#include <SerialBridge.hpp>
#include <LinuxHardwareSerial.hpp>

#include "./Gesture.h"
#include "./Controller.hpp"
#include "./MovementFeedback.hpp"

#define SERIAL_PATH "/dev/serial/by-path/pci-0000:00:14.0-usb-0:4:1.2"

#define CONTROLLER_ID 0
#define GESTURE_ID 1
#define MOVEMENT_FEEDBACK_ID 5

SerialDev *dev = new LinuxHardwareSerial(SERIAL_PATH, B115200);
SerialBridge serial(dev, 1024);

Gesture gesture_msg;
// DebugMessage debug_msg;
Controller controller_msg;
MovementFeedback movement_feedback_msg;

void callbackUserAction(const robocon2022_essentials_msgs::UserAction& userAction)
{
  gesture_msg.data.type = userAction.action_id - 2;
  serial.write(GESTURE_ID);
}

void callbackController(const robocon2022_essentials_msgs::Controller& msg)
{
  controller_msg.data.movement_mode = (int8_t)msg.movement_mode;
  controller_msg.data.movement.x = msg.movement_vel.linear.x;
  controller_msg.data.movement.y = msg.movement_vel.linear.y;
  controller_msg.data.movement.z = msg.movement_vel.angular.z;
  controller_msg.data.all_reload = msg.all_reload;
  controller_msg.data.shooter.num = (int8_t)msg.shooter_num;
  controller_msg.data.shooter.power = msg.shooter_power;
  controller_msg.data.shooter.action = (int8_t)msg.shooter_action;

  serial.write(CONTROLLER_ID);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "robocon2022_essentials_serial");
  ros::NodeHandle nh;
  ros::Rate loop_rate(50);

  //  SerialBridge
  serial.add_frame(GESTURE_ID, &gesture_msg);
  serial.add_frame(CONTROLLER_ID, &controller_msg);
  serial.add_frame(MOVEMENT_FEEDBACK_ID, &movement_feedback_msg);

  //  subscriber
  ros::Subscriber user_action_sub_ = nh.subscribe("/essentials_detection/user_action", 10, callbackUserAction);
  ros::Subscriber controller_sub_ = nh.subscribe("/essentials_controller/controller", 10, callbackController);

  while(ros::ok())
  {
    if(serial.update() == 0)
    {
      if(movement_feedback_msg.was_updated())
      {
      }
    }

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}