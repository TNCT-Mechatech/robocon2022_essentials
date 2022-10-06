#include <ros/ros.h>

#include <iostream>
#include <cstdio>
#include <map>
#include <string>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

//	Message
#include <geometry_msgs/Point32.h>
#include <body_tracker_msgs/BodyTracker.h>
#include <body_tracker_msgs/BodyTrackerArray.h>
#include <body_tracker_msgs/Gesture.h>
#include <body_tracker_msgs/Gestures.h>
#include <body_tracker_msgs/Skeleton.h>
#include <robocon2022_essentials_msgs/UserAction.h>
#include "sensor_msgs/Image.h"

#include <image_transport/image_transport.h>

#define MATRIX_ROW 848
#define MATRIX_COL 480
#define REALSENSE_D435_FOV_X 1.50098
#define REALSENSE_D435_FOV_Y 0.994838
#define MAXIMUM_DEPTH 2.5

class visualizer_node
{
public:
  visualizer_node()
  {
    ROS_INFO("Node initializing");

    ros::NodeHandle pnh("~");

    ros::Subscriber image_sub_ = pnh.subscribe("/camera/color/image", 10, &visualizer_node::callback_image, this);
    ros::Subscriber body_tracker_array_sub_ = pnh.subscribe("/body_tracker_array/position", 10,  &visualizer_node::callback_body_tracker, this);
    ros::Subscriber user_action_sub_ = pnh.subscribe("/essentials_detection/user_action", 10,  &visualizer_node::callback_user_action, this);
    ros::Subscriber skelton_sub_ = pnh.subscribe("/body_tracker/skeleton", 10,  &visualizer_node::callback_skelton, this);


    //  publisher
    image_transport::ImageTransport it(pnh);
    image_pub_ = it.advertise("/essentials_visualizer/color/image/", 10);

    ros::spin();
  }

  ~visualizer_node()
  {
    ROS_INFO("shutdown");
  }

  void run()
  {
    ros::spin();
  }

  //////////////////////
  //  Callback
  //  Image callback
  void callback_image(const sensor_msgs::Image& msg)
  {
    // ROS_INFO("get new Image");

    try{
      last_image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
      //  call drawing function
      drawing_frame();
    }
    catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
    }
  }

  //  BodyTracker callback
  void callback_body_tracker(const body_tracker_msgs::BodyTrackerArray& msg)
  {
    // ROS_INFO("get new BodyTracker");
    last_body_tracker_array = msg;
  }

  //  UserAction callback
  void callback_user_action(const robocon2022_essentials_msgs::UserAction& msg)
  {
    ROS_INFO("get new UserAction");

    throw_display_counter_ = 40;
  }

  //  Skeleton callback
  void callback_skelton(const body_tracker_msgs::Skeleton& msg)
  {
    // ROS_INFO("get new Skeleton");

    skeleton_map[msg.body_id] = msg;
  }

  //  drawing
  void drawing_frame()
  {
    // ROS_INFO("drawing");
    /**
     * Note: Mat's notation is not (x,y), but (row,col). 
     * O    row
     *   * - - - >
     * c |
     * o |
     * l |
     *   _
     */
    cv::Mat image = last_image.clone();

    body_tracker_msgs::BodyTracker *center_body = nullptr;

    //  compare
    double min_distance = 100;
    for (body_tracker_msgs::BodyTracker& any_body : last_body_tracker_array.detected_list)
    {
      // ROS_INFO("CLOCK");
      double distance = abs(any_body.position2d.x);
      if (distance < min_distance)
      {
        // ROS_INFO("SWAP");
        //  swap
        center_body = &any_body;
        min_distance = distance;
      }
    }

    if (center_body != nullptr) {
    // draw
    for (body_tracker_msgs::BodyTracker& any_body : last_body_tracker_array.detected_list)
    {
      try
      {
        // ROS_INFO("Draw body id: %d", any_body.body_id);

        auto finded_value = skeleton_map.find(any_body.body_id);
        if(finded_value == skeleton_map.end()) {
          ROS_INFO("Skip due to NOT FOUND");
          return;
        }
        body_tracker_msgs::Skeleton center_skeleton = finded_value->second;

        // center_skeleton
        cv::Point center_rectangle[2];
        //  top and left
        center_rectangle[0] = cv::Point(format_x(center_skeleton.joint_position_right_hand.x, 0.5), format_y(center_skeleton.joint_position_head.y - 0.2, 0.5));
        //  bottom and right
        center_rectangle[1] = cv::Point(format_x(center_skeleton.joint_position_left_hand.x, 0.5), format_y(center_skeleton.joint_position_spine_mid.y + 0.2, 0.5));

        //  Color
        cv::Scalar human_color;
        if (any_body.body_id == center_body->body_id) {
          human_color = cv::Scalar(0, 255, 0);
        }
        else {
          //  gray
          human_color = cv::Scalar(255, 165, 0);
        }

        //  draw
        cv::rectangle(
          image,
          center_rectangle[0],
          center_rectangle[1],
          human_color,
          3
        );

        //  draw user id        
        cv::rectangle(
          image,
          center_rectangle[0],
          cv::Point(center_rectangle[0].x + (MATRIX_ROW * 0.15) , center_rectangle[0].y - (0.045 * MATRIX_COL)),
          human_color,
          -1
        );

        cv::Scalar font_color = cv::Scalar(255, 0, 0);
        std::string base_text("UserID: ");
        std::string user_text = base_text + std::to_string(any_body.body_id);
        cv::putText(
          image,
          user_text,
          center_rectangle[0],
          cv::FONT_HERSHEY_PLAIN,
          1.5,
          font_color,
          2
        );

        if(any_body.body_id == center_body->body_id)
        {
          //  user depth
          cv::Scalar depth_back_color = cv::Scalar(0, 255, 0);
          cv::Scalar depth_color = cv::Scalar(0, 0, 0);
          // std::string meter_text(" M");
          // std::string depth_text = std::to_string(any_body.position2d.z) + meter_text;
          std::ostringstream oss;
          oss << std::fixed << std::setprecision(3) << any_body.position2d.z << " M";
          std::string depth_text = oss.str();
          cv::rectangle(
            image,
            cv::Point(0, 0),
            cv::Point(MATRIX_ROW * 0.2 , MATRIX_COL * 0.1),
            depth_back_color,
            -1
          );
          cv::putText(
            image,
            depth_text,
            cv::Point(MATRIX_ROW * 0.01, MATRIX_COL * 0.07),
            cv::FONT_HERSHEY_PLAIN,
            2,
            depth_color,
            2
          );

          //  Depth Frame
          cv::Scalar frame_color;
          if(any_body.position2d.z <= MAXIMUM_DEPTH)
          {
            frame_color = cv::Scalar(0, 255, 0);
          }
          else
          {
            frame_color = cv::Scalar(255, 0, 0);
          }
          //  render
          cv::rectangle(
            image,
            cv::Point(0, 0),
            cv::Point(MATRIX_ROW * 1 , MATRIX_COL * 1),
            frame_color,
            14
          );
        }
      } catch (std::out_of_range& oor)
      {
        ROS_INFO("Not found skelton");
      }

    }
    }

    if (throw_display_counter_ > 0)
    {
      //  Display status of throwing
      cv::rectangle(
        image,
        cv::Point(0.3*MATRIX_ROW, 0.1*MATRIX_COL),
        cv::Point(0.7*MATRIX_ROW, 0.3*MATRIX_COL),
        cv::Scalar(255, 255, 255),
       -1
      );

      cv::putText(
        image,
        std::string("Throw!!"),
        cv::Point(0.35*MATRIX_ROW, 0.25*MATRIX_COL),
        cv::FONT_HERSHEY_PLAIN,
        4.5,
        cv::Scalar(0, 0, 0),
        5
      );

      throw_display_counter_ --;
    }
    

    // ROS_INFO("PUBLISH");
    //  convert
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
    //  publish image topic
    image_pub_.publish(msg);
  }

  //////////////////
  //  Util
  //  get center body
  bool get_center_body(body_tracker_msgs::BodyTracker *center_body)
  {
    ROS_INFO("SEARCH CNETER BODY, count: %d", (int) last_body_tracker_array.detected_list.size());

    //  reset
    center_body = nullptr;

    //  compare
    double min_distance = 100;
    for (body_tracker_msgs::BodyTracker any_body : last_body_tracker_array.detected_list)
    {
      ROS_INFO("CLOCK");

      double distance = sqrt(pow(any_body.position2d.x, 2.0) + pow(any_body.position2d.y, 2.0));
      if (distance < min_distance)
      {
        ROS_INFO("SWAP");
        //  swap
        center_body = &any_body;
        min_distance = distance;
      }
    }

    return center_body != nullptr;
  }

  //  Position converter X
  double format_x(double value, double max_value)
  {
    return (value + max_value) * MATRIX_ROW;
  }

  //  Position converter Y
  double format_y(double value, double max_value)
  {
    // return MATRIX_COL - ((value + max_value) * MATRIX_COL);
    return (value + max_value) * MATRIX_COL;
  }

private:
  //  image publisher
  image_transport::Publisher image_pub_;
  //  received image
  cv::Mat last_image;
  
  //  store skelton with user id
  std::map<int, body_tracker_msgs::Skeleton> skeleton_map;
  //  BodyTrackerArray
  body_tracker_msgs::BodyTrackerArray last_body_tracker_array;

  int throw_display_counter_ = 0;

};



int main(int argc, char** argv){
  ros::init(argc, argv, "visualizer");
  visualizer_node node = visualizer_node();
  // node.run();

  return 0;
}