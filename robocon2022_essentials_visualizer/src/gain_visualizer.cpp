#include <ros/ros.h>

#include <string>
#include <cstdio>
#include <vector>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

//	Message
#include <std_msgs/Float32.h>

//  publisher
#include <image_transport/image_transport.h>

#define MATRIX_ROW 1024
#define MATRIX_COL 600


template <typename ... Args>
std::string format(const std::string& fmt, Args ... args )
{
    size_t len = std::snprintf( nullptr, 0, fmt.c_str(), args ... );
    std::vector<char> buf(len + 1);
    std::snprintf(&buf[0], len + 1, fmt.c_str(), args ... );
    return std::string(&buf[0], &buf[0] + len);
}

class gain_visualizer_node
{
public:
  gain_visualizer_node()
  {
    ROS_INFO("Node initializing");

    _gain = 0.000;

    ros::NodeHandle pnh("~");
    ros::Rate loop_rate(25);

    //  gain subscriber
    ros::Subscriber gain_sub_ = pnh.subscribe("/essentials_serial/gain", 10, &gain_visualizer_node::callback_gain, this);

    //  publisher
    image_transport::ImageTransport it(pnh);
    image_pub_ = it.advertise("/essentials_gain_visualizer/color/image/", 10);

    while(ros::ok())
    {
      drawing_frame();

      ros::spinOnce();
      loop_rate.sleep();
    }

  }

  ~gain_visualizer_node()
  {
    ROS_INFO("shutdown");
  }

  //////////////////////
  //  Callback
  //  gain callback
  void callback_gain(const std_msgs::Float32& msg)
  {
    ROS_INFO("gain: %.3f", msg.data);

    _gain = msg.data;
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
    cv::Mat image(MATRIX_ROW, MATRIX_COL, CV_64FC(3));

    //  background color
    image = cv::Scalar(255, 255, 255);

    //  text
    cv::Scalar font_color = cv::Scalar(0, 0, 0);
    std::string text = format("%.3f", _gain);
    cv::putText(
      image,
      text,
      cv::Point(100, 200),
      cv::FONT_HERSHEY_PLAIN,
      10,
      font_color,
      2
    );

    //  convert
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
    //  publish image topic
    image_pub_.publish(msg);
  }

private:
  //  image publisher
  image_transport::Publisher image_pub_;
  
  float _gain;
};



int main(int argc, char** argv){
  ros::init(argc, argv, "gain_visualizer");
  gain_visualizer_node node = gain_visualizer_node();

  return 0;
}