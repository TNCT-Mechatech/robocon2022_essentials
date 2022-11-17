#include <ros/ros.h>

#include <string>
#include <cstdio>
#include <vector>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

//	Message
#include <robocon2022_essentials_msgs/Controller.h>

//  publisher
#include <image_transport/image_transport.h>

#define MATRIX_COL 1920
#define MATRIX_ROW 1080

using robocon2022_essentials_msgs::Controller;

template <typename ... Args>
std::string format(const std::string& fmt, Args ... args )
{
    size_t len = std::snprintf( nullptr, 0, fmt.c_str(), args ... );
    std::vector<char> buf(len + 1);
    std::snprintf(&buf[0], len + 1, fmt.c_str(), args ... );
    return std::string(&buf[0], &buf[0] + len);
}

class panel_visualizer_node
{
public:
  panel_visualizer_node()
  {
    ROS_INFO("Node initializing");

    ros::NodeHandle pnh("~");
    ros::Rate loop_rate(20);

    //  gain subscriber
    ros::Subscriber gain_sub_ = pnh.subscribe("/essentials_controller/controller", 10, &panel_visualizer_node::callback_controller, this);

    //  publisher
    image_transport::ImageTransport it(pnh);
    image_pub_ = it.advertise("/essentials_panel_visualizer/color/image/", 10);

    while(ros::ok())
    {
      drawing_frame();

      ros::spinOnce();
      loop_rate.sleep();
    }

  }

  ~panel_visualizer_node()
  {
    ROS_INFO("shutdown");
  }

  //  Callback
  void callback_controller(const Controller& msg)
  {
    controller_msg = msg;
  }

  //  drawing
  void drawing_frame()
  {
    /**
     * Note: Mat's notation is not (x,y), but (row,col). 
     * O    row
     *   * - - - >
     * c |
     * o |
     * l |
     *   _
     */
    cv::Mat image(MATRIX_ROW, MATRIX_COL, CV_8UC(3));

    //  background color
    image = cv::Scalar(255, 255, 255);

    //  color
    cv::Scalar black_color = cv::Scalar(0, 0, 0);
    cv::Scalar green_color = cv::Scalar(0, 255, 0);
    cv::Scalar red_color = cv::Scalar(255, 0, 0);

    //  status
    std::string status_str = std::string("Status: ") + (controller_msg.emergency_switch ? std::string("Active") : std::string("Stopped"));
    put_text(
      image,
      status_str,
      controller_msg.emergency_switch ? green_color : red_color,
      1
    );

    //  Movement mode
    std::string mode_str("Mode: ");
    if (controller_msg.movement_mode == 0)
    {
      mode_str += std::string("Fast");
    }
    else if (controller_msg.movement_mode == 1)
    {
      mode_str += std::string("Normal");
    }
    else if (controller_msg.movement_mode == 2)
    {
      mode_str += std::string("Slow");
    }
    
    put_text(
      image,
      mode_str,
      black_color,
      3
    );

    /*
    //  Movement
    put_text(
      image,
      format("X:%5d%", (int)(controller_msg.movement_vel.linear.x * 100)),
      black_color,
      2
    );
    put_text(
      image,
      format("Y:%5d%", (int)(controller_msg.movement_vel.linear.y * 100)),
      black_color,
      3
    );
    put_text(
      image,
      format("Z:%5d%", (int)(controller_msg.movement_vel.angular.z * 100)),
      black_color,
      4
    );
    */

    //  Shooter
    std::string shooter_selected("Shooter: ");
    if(controller_msg.shooter_num == 0)
    {
      shooter_selected += std::string("Child");
    }
    else if(controller_msg.shooter_num == 1)
    {
      shooter_selected += std::string("Parent RIGHT");
    }
    else if(controller_msg.shooter_num == 2)
    {
      shooter_selected += std::string("Parent LEFT");
    }

    put_text(
      image,
      shooter_selected,
      black_color,
      4
    );
    put_text(
      image,
      format("Power:%5.1lf%", controller_msg.shooter_power * 100),
      black_color,
      5
    );

    //  Shooter state
    std::string shooter_state_str("Status: ");
    if (controller_msg.shooter_action == 1)
    {
      shooter_state_str += std::string("UP");
    }
    else if (controller_msg.shooter_action == 2)
    {
      shooter_state_str += std::string("DOWN");
    }
    else if (controller_msg.shooter_action == 3)
    {
      shooter_state_str += std::string("SHOOT!!");
    }

    put_text(
      image,
      shooter_state_str,
      black_color,
      6
    );

    if(controller_msg.all_reload)
    {
      put_text(
        image,
        std::string("All reload!"),
        black_color,
        7
      );
    }
    

    //  convert
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
    //  publish image topic
    image_pub_.publish(msg);
  }

  void put_text(cv::Mat image, std::string text, cv::Scalar color, int line = 1)
  {
    cv::putText(
      image,
      text,
      cv::Point(40, line * 100),
      cv::FONT_HERSHEY_PLAIN,
      7,
      color,
      4
    );
  }
private:
  //  image publisher
  image_transport::Publisher image_pub_;

  Controller controller_msg;
};



int main(int argc, char** argv){
  ros::init(argc, argv, "panel_visualizer");
  panel_visualizer_node node = panel_visualizer_node();

  return 0;
}