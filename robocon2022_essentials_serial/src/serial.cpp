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
#include <robocon2022_essentials_msgs/Wheel4.h>

//  SerialBridge
#include <SerialBridge.hpp>
#include <LinuxHardwareSerial.hpp>

#include "./Gesture.h"
#include "./Controller.hpp"
#include "./MovementFeedback.hpp"
#include "./DebugMessage.h"

#define SERIAL_PATH "/dev/serial/by-path/pci-0000:00:14.0-usb-0:4:1.2"

#define CONTROLLER_ID 0
#define GESTURE_ID 1
#define MOVEMENT_FEEDBACK_ID 5
#define DEBUG_ID 6

SerialDev *dev = new LinuxHardwareSerial(SERIAL_PATH, B115200);
SerialBridge serial(dev, 1024);

Gesture gesture_msg;
DebugMessage debug_msg;
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
  serial.add_frame(DEBUG_ID, &debug_msg);

  //  subscriber
  ros::Subscriber user_action_sub_ = nh.subscribe("/essentials_detection/user_action", 10, callbackUserAction);
  ros::Subscriber controller_sub_ = nh.subscribe("/essentials_controller/controller", 10, callbackController);
  //  Publisher
  ros::Publisher debug_pub = nh.advertise<std_msgs::String>("/essentials_serial/debug", 1000);
  ros::Publisher movenet_feedback_pub = nh.advertise<robocon2022_essentials_msgs::Wheel4>("/essentials_serial/feedback", 1000);

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

      if(movement_feedback_msg.was_updated())
      {
        robocon2022_essentials_msgs::Wheel4 msg;
        msg.target.v1 = movement_feedback_msg.data.target.v1;
        msg.target.v2 = movement_feedback_msg.data.target.v2;
        msg.target.v3 = movement_feedback_msg.data.target.v3;
        msg.target.v4 = movement_feedback_msg.data.target.v4;

        msg.present.v1 = movement_feedback_msg.data.output.v1;
        msg.present.v2 = movement_feedback_msg.data.output.v2;
        msg.present.v3 = movement_feedback_msg.data.output.v3;
        msg.present.v4 = movement_feedback_msg.data.output.v4;

        movenet_feedback_pub.publish(msg);
      }
    }

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
