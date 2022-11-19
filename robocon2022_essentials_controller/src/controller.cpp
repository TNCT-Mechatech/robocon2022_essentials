#include <ros/ros.h>

#include <sensor_msgs/Joy.h>
#include <std_msgs/Bool.h>
#include <robocon2022_essentials_msgs/Controller.h>

using robocon2022_essentials_msgs::Controller;

#define SHOOTER_POWER_FAST_DIFFERENCE 0.10
#define SHOOTER_POWER_SLOW_DIFFERENCE 0.01

class joy_controller_node
{
public:

  joy_controller_node()
  {
    ROS_INFO("Initialize Joy Controller Node...");

    ros::NodeHandle pnh("~");
    //  10Hz
    ros::Rate loop_rate(10);

    //  init variable
    init_native_controller();

    _emergency_switch = false;

    //  subscriber
    ros::Subscriber joy_sub_ = pnh.subscribe("/joy", 10, &joy_controller_node::callbackJoy, this);
    ros::Subscriber emergency_sub_ = pnh.subscribe("/essentials_controller/emergency", 10, &joy_controller_node::callbackEmergency, this);


    //  publisher
    controller_pub = pnh.advertise<Controller>("/essentials_controller/controller", 1000);

    while(ros::ok())
    {
      //  publish
      publish_controller();

      ros::spinOnce();
      loop_rate.sleep();
    }

    ros::spin();

    ROS_INFO("Completed initialization");
  }

  ~joy_controller_node()
  {

  }

  void init_native_controller()
  {
    _previous_controller = {
      0.0, 0.0, 0.0,
      false, false, false, false,
      false, false, false, false,
      false, false,
      false, false
    };

    _present_controller = {
      0.0, 0.0, 0.0,
      false, false, false, false,
      false, false, false, false,
      false, false,
      false, false
    };

  }

  void callbackEmergency(const std_msgs::Bool& msg)
  {
    _emergency_switch = msg.data;
  }

  void callbackJoy(const sensor_msgs::Joy& msg)
  {
    // ROS_INFO("subscribed joy");
    _need_publish = false;

    //  init message
    controller_msg.all_reload = false;
    controller_msg.shooter_action = 0;

    //  emergency
    controller_msg.emergency_switch = _emergency_switch;

    //  convert into native controller type
    convert_native_controller(msg);

    //  set movement variable
    controller_msg.movement_vel.linear.x = _present_controller.x;
    controller_msg.movement_vel.linear.y = _present_controller.y;
    controller_msg.movement_vel.angular.z = _present_controller.theta;

    //  check difference of previous status
    /////////////////////////////
    //  Left cross key part
    /**
     * @brief Left cross UP key: increase shooter power
     */
    if(!_previous_controller.lc_up && _present_controller.lc_up) {
      float power = controller_msg.shooter_power;

      //  increase
      power += _present_controller.lb_1 ? SHOOTER_POWER_FAST_DIFFERENCE: SHOOTER_POWER_SLOW_DIFFERENCE;

      //  limit
      controller_msg.shooter_power = power > 1.0 ? 1.0: power;
    }

    /**
     * @brief Left cross DOWN key: discrease shooter power
     */
    if(!_previous_controller.lc_down && _present_controller.lc_down) {
      float power = controller_msg.shooter_power;

      //  increase
      power -= _present_controller.lb_1 ? SHOOTER_POWER_FAST_DIFFERENCE: SHOOTER_POWER_SLOW_DIFFERENCE;

      //  limit
      controller_msg.shooter_power = power < 0.0 ? 0.0: power;
    }

    /**
     * @brief Left cross LEFT key: change emotion
     */
    if(!_previous_controller.lc_left && _present_controller.lc_left) {
      int mode = controller_msg.face + 1;

      //  limitter
      controller_msg.face = mode >= 2 ? 0: mode;
    }

    /////////////////////////////
    //  Right cross key part
    /**
     * @brief Right cross LEFT key: all reload
     */
    if(!_previous_controller.rc_left && _present_controller.rc_left) {
      controller_msg.all_reload = true;

      //  publish
      _need_publish = true;
    }

    /**
     * @brief Right cross RIGHT key: change movement speed
     */
    if(!_previous_controller.rc_right && _present_controller.rc_right) {
      int mode = controller_msg.movement_mode + 1;
      //  limitter
      controller_msg.movement_mode = mode >= 3 ? 0: mode;

      //  publish
      _need_publish = true;
    }

    /////////////////////////////
    //  Right button part
    /**
     * @brief Right button R1: change shooter
     */
    if(!_previous_controller.rb_1 && _present_controller.rb_1) {
      int mode = controller_msg.shooter_num + 1;
      //  limitter
      controller_msg.shooter_num = mode >= 3 ? 0: mode;

      //  publish
      _need_publish = true;
    }

    /**
     * @brief Right button R2: shoot
     */
    if(!_previous_controller.rb_2 && _present_controller.rb_2) {
      controller_msg.shooter_action = 3;

      //  publish
      _need_publish = true;
    }

    /////////////////////////////
    //  Shooter actions
    if(controller_msg.shooter_action != 3)
    {
      if(_present_controller.rc_up && _present_controller.rc_down)
      {
        //  No! No! Don't press the combination of UP + DOWN
      }
      else if (_present_controller.rc_up)
      {
        controller_msg.shooter_action = 1;
      }
      else if (_present_controller.rc_down)
      {
        controller_msg.shooter_action = 2;
      }
    }

    //  update status
    _previous_controller = _present_controller;

    //  if need to publish
    if(_need_publish)
    {
      publish_controller();
    }
  }

  void publish_controller()
  {
    controller_pub.publish(controller_msg);
  }

  void convert_native_controller(const sensor_msgs::Joy msg)
  {
    //  joystic
    _present_controller.x = -1 * msg.axes[0];
    _present_controller.y = msg.axes[1];
    _present_controller.theta = -1 * msg.axes[3];
    //  Left cross buttons
    _present_controller.lc_up = msg.axes[7] >= 1.0 ? true : false;
    _present_controller.lc_down = msg.axes[7] <= -1.0 ? true : false;
    _present_controller.lc_left = msg.axes[6] >= 1.0 ? true : false;
    _present_controller.lc_right = msg.axes[6] <= -1.0 ? true : false;
    //  Right cross buttons
    _present_controller.rc_up = msg.buttons[2];
    _present_controller.rc_down = msg.buttons[0];
    _present_controller.rc_left = msg.buttons[3];
    _present_controller.rc_right = msg.buttons[1];
    //  Left button
    _present_controller.lb_1 = msg.buttons[4];
    _present_controller.lb_2 = msg.buttons[6];
    //  Right button
    _present_controller.rb_1 = msg.buttons[5];
    _present_controller.rb_2 = msg.buttons[7];
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

  //  native controller
  native_controller_t _previous_controller;
  native_controller_t _present_controller;

  //  publish flag
  bool _need_publish;

  //  emergency switch
  bool _emergency_switch;

  //  Publish message
  Controller controller_msg;
  //  Publisher
  ros::Publisher controller_pub;

};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "robocon2022_essentials_controller");

  //  init node
  joy_controller_node joy_controller = joy_controller_node();

  return 0;
}