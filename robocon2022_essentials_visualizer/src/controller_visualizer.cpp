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

class controller_visualizer_node
{
public:
  controller_visualizer_node()
  {
    ROS_INFO("Node initializing");

    ros::NodeHandle pnh("~");
    ros::Rate loop_rate(10);

    //  gain subscriber
    ros::Subscriber gain_sub_ = pnh.subscribe("/essentials_controller/controller", 10, &controller_visualizer_node::callback_controller, this);

    //  publisher
    image_transport::ImageTransport it(pnh);
    image_pub_ = it.advertise("/essentials_controller_visualizer/color/image/", 10);

    while(ros::ok())
    {
      drawing_frame();

      ros::spinOnce();
      loop_rate.sleep();
    }

  }

  ~controller_visualizer_node()
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
      1
    );

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

    //  Shooter
    put_text(
      image,
      format("shooter: %d", controller_msg.shooter_num),
      black_color,
      6
    );
    put_text(
      image,
      format("Power:%5.1lf%", controller_msg.shooter_power * 100),
      black_color,
      7
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
      8
    );

    if(controller_msg.all_reload)
    {
      put_text(
        image,
        std::string("All reload!"),
        black_color,
        10
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
  ros::init(argc, argv, "controller_visualizer");
  controller_visualizer_node node = controller_visualizer_node();

  return 0;
}