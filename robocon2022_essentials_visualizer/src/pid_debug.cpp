#include <ros/ros.h>

#include <string>
#include <cstdio>
#include <vector>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

//	Message
#include <robocon2022_essentials_msgs/Wheel4.h>

//  publisher
#include <image_transport/image_transport.h>

#define MATRIX_COL 1920
#define MATRIX_ROW 1080

using robocon2022_essentials_msgs::Wheel4;

class pid_debug_visualizer_node
{
public:
  pid_debug_visualizer_node()
  {
    ROS_INFO("Node initializing");

    ros::NodeHandle pnh("~");
    ros::Rate loop_rate(20);

    //  gain subscriber
    ros::Subscriber gain_sub_ = pnh.subscribe("/essentials_serial/feedback", 10, &pid_debug_visualizer_node::callback_wheel, this);

    //  publisher
    image_transport::ImageTransport it(pnh);
    image_pub_ = it.advertise("/essentials_pid_debug_visualizer/color/image/", 10);

    while(ros::ok())
    {
      drawing_frame();

      ros::spinOnce();
      loop_rate.sleep();
    }

  }

  ~pid_debug_visualizer_node()
  {
    ROS_INFO("shutdown");
  }

  //  Callback
  void callback_wheel(const Wheel4& msg)
  {
    wheel_msg = msg;
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
    cv::Scalar red_color = cv::Scalar(0, 0, 255);

    //  w1
    put_metor(
      image,
      wheel_msg.target.v1,
      red_color,
      wheel_msg.present.v1,
      green_color,
      1
    );
    //  w2
    put_metor(
      image,
      wheel_msg.target.v2,
      red_color,
      wheel_msg.present.v2,
      green_color,
      2
    );
    //  w3
    put_metor(
      image,
      wheel_msg.target.v3,
      red_color,
      wheel_msg.present.v3,
      green_color,
      3
    );
    //  w4
    put_metor(
      image,
      wheel_msg.target.v4,
      red_color,
      wheel_msg.present.v4,
      green_color,
      4
    );

    //  convert
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
    //  publish image topic
    image_pub_.publish(msg);
  }

  void put_metor(cv::Mat image, double target, cv::Scalar target_color, double present, cv::Scalar present_color, int line = 1)
  {
    int start_col = 100;
    int start_row = (line-1) * 400 + 100;
    double range = 9.0;

    //  10 px
    int thickness = 5;
    //  100 px
    int width = 100;

    //  zero line
    int zero_col = calculate_col(range, 0);
    cv::Point zero_point(start_row, zero_col);
    cv::Point zero_end_point(start_row + width, zero_col);
    cv::line(
      image,
      zero_point,
      zero_end_point,
      (0,0,0),
      thickness
    );

    //  target
    int target_col = calculate_col(range, target);
    cv::Point target_point(start_row, target_col);
    cv::Point target_end_point(start_row + width, target_col);
    cv::line(
      image,
      target_point,
      target_end_point,
      target_color,
      thickness
    );

    //  present
    int present_col = calculate_col(range, present);
    cv::Point present_point(start_row, present_col);
    cv::Point present_end_point(start_row + width, present_col);
    cv::line(
      image,
      present_point,
      present_end_point,
      present_color,
      thickness
    );
  }

  int calculate_col(double range, double value)
  {
    int height = 1000;
    double duty = value / range;

    return (int) ((-1 * height/2 * duty) + 400);
  }

private:
  //  image publisher
  image_transport::Publisher image_pub_;

  Wheel4 wheel_msg;
};



int main(int argc, char** argv){
  ros::init(argc, argv, "pid_debug_visualizer");
  pid_debug_visualizer_node node = pid_debug_visualizer_node();

  return 0;
}