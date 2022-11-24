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

#define UPSTREAM_CONTROLLER_TIMEOUT 2.0

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

    //  subscriber
    ros::Subscriber controller_sub_ = pnh.subscribe("/essentials_controller/controller", 10, &panel_visualizer_node::callback_controller, this);
    ros::Subscriber sub_controller_sub_ = pnh.subscribe("/essentials_serial/controller_pub", 10, &panel_visualizer_node::callback_sub_controller, this);

    //  publisher
    image_transport::ImageTransport it(pnh);
    image_pub_ = it.advertise("/essentials_panel_visualizer/color/image/", 10);

    last_rx_upstream = ros::Time::now().toSec();

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

  void callback_sub_controller(const Controller& msg)
  {
    sub_controller_msg = msg;
    last_rx_upstream = ros::Time::now().toSec();
  }

  //  drawing
  void drawing_frame()
  {
    //  controller
    Controller controller = ros::Time::now().toSec() - last_rx_upstream > UPSTREAM_CONTROLLER_TIMEOUT ? controller_msg : sub_controller_msg;

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
    cv::Scalar green_color = cv::Scalar(0, 191, 0);
    cv::Scalar red_color = cv::Scalar(0, 0, 218);
    cv::Scalar orange_color = cv::Scalar(0, 165, 255);
    cv::Scalar blue_color = cv::Scalar(255, 102, 0);

    //  status
    std::string status_str = std::string("Status: ") + (controller.emergency_switch ? std::string("Active") : std::string("Stopped"));
    put_text(
      image,
      status_str,
      controller.emergency_switch ? green_color : red_color,
      1
    );

    //  Movement mode
    cv::Scalar mode_color = black_color;
    std::string mode_str("Mode: ");
    if (controller.movement_mode == 0)
    {
      mode_str += std::string("Fast");
      mode_color = red_color;
    }
    else if (controller.movement_mode == 1)
    {
      mode_str += std::string("Normal");
      mode_color = orange_color;
    }
    else if (controller.movement_mode == 2)
    {
      mode_str += std::string("Slow");
      mode_color = blue_color;
    }
    
    put_text(
      image,
      mode_str,
      mode_color,
      3
    );

    //  Shooter
    std::string shooter_selected("Shooter: ");
    if(controller.shooter_num == 0)
    {
      shooter_selected += std::string("Child");
    }
    else if(controller.shooter_num == 1)
    {
      shooter_selected += std::string("Parent RIGHT-->>");
    }
    else if(controller.shooter_num == 2)
    {
      shooter_selected += std::string("<<--Parent LEFT");
    }

    put_text(
      image,
      shooter_selected,
      black_color,
      4
    );
    put_text(
      image,
      format("Power:%5.1lf%", controller.shooter_power * 100),
      black_color,
      5
    );

    //  Face state
    std::string face_str("Face: ");
    if (controller.face == 1)
    {
      face_str += std::string("Normal :)");
    }
    else if (controller.face == 2)
    {
      face_str += std::string("Funny :D");
    }
    else if (controller.face == 3)
    {
      face_str += std::string("Sadly :(");
    }
    put_text(
      image,
      face_str,
      black_color,
      6
    );

    //  Shooter state
    std::string shooter_state_str("Status: ");
    if (controller.shooter_action == 1)
    {
      shooter_state_str += std::string("UP");
    }
    else if (controller.shooter_action == 2)
    {
      shooter_state_str += std::string("DOWN");
    }
    else if (controller.shooter_action == 3)
    {
      shooter_state_str += std::string("SHOOT!!");
    }

    put_text(
      image,
      shooter_state_str,
      black_color,
      7
    );

    if(controller.all_reload)
    {
      put_text(
        image,
        std::string("All reload!"),
        black_color,
        8
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
      10,
      cv::LINE_AA
    );
  }
private:
  //  image publisher
  image_transport::Publisher image_pub_;

  Controller controller_msg;
  Controller sub_controller_msg;
  double last_rx_upstream;
};



int main(int argc, char** argv){
  ros::init(argc, argv, "panel_visualizer");
  panel_visualizer_node node = panel_visualizer_node();

  return 0;
}