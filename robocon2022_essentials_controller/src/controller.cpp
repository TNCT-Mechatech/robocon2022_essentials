#include <ros/ros.h>

#include <sensor_msgs/Joy.h>


class joy_controller_node
{
public:

  joy_controller_node()
  {
    ROS_INFO("Initialize Joy Controller Node...");

    ros::NodeHandle pnh("~");

    //  subscriber
    ros::Subscriber joy_sub_ = pnh.subscribe("/joy", 10, &joy_controller_node::callbackJoy, this);

    //  publisher


    ros::spin();

    ROS_INFO("Completed initialization");
  }

  ~joy_controller_node()
  {

  }

  void callbackJoy(const sensor_msgs::Joy& msg)
  {
    ROS_INFO("subscribed joy");
  }

private:

  /////////////////////////
  //  Type Definition

  /**
   * @brief Native Controller (what is unformatted joy controller variables)
   */
  typedef struct NativeControllerType
  {
    //  Joy stick
    double x;
    double y;
    double theta;

    //  Left cross buttons
    bool lc_up;
    bool lc_left;
    bool lc_right;
    bool lc_down;

    //  Right cross buttons
    bool rc_up;
    bool rc_left;
    bool rc_right;
    bool rc_down;

    //  Left button
    bool lb_1;
    bool lb_2;

    //  Right button
    bool rb_1;
    bool rb_2;
  } native_controller_t;


  /////////////////////////
  //  Private variables


};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "robocon2022_essentials_controller");

  //  init node
  joy_controller_node joy_controller = joy_controller_node();

  return 0;
}