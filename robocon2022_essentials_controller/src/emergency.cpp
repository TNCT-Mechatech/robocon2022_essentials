#include <ros/ros.h>
#include <std_msgs/Bool.h>

#ifdef __arm__

#include <wiringPi.h>

#define PIN 15
#endif

int main(int argc, char** argv)
{
  ros::init(argc, argv, "robocon2022_essentials_emergency");
  ros::NodeHandle pnh("~");
  ros::Rate loop_rate(5);

  ros::Publisher emergency_pub = pnh.advertise<std_msgs::Bool>("/essentials_controller/emergency", 1000);

  #ifdef __arm__

  wiringPiSetup();
  pinMode(PIN, INPUT);
  pullUpDnControl(PIN, PUD_DOWN);

  while(ros::ok())
  {
    std_msgs::Bool msg;
    
    msg.data = digitalRead(PIN);

    emergency_pub.publish(msg);


    ros::spinOnce();
    loop_rate.sleep();
  }

  #else
  //  Nothing
  #endif
  return 0;
}