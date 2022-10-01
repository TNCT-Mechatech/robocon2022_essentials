#include <ros/ros.h>
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"

#include <iostream>
#include <deque>
#include <math.h>

//	Message
#include <geometry_msgs/Point32.h>
#include <body_tracker_msgs/BodyTracker.h>
#include <body_tracker_msgs/BodyTrackerArray.h>
#include <body_tracker_msgs/Gesture.h>
#include <body_tracker_msgs/Gestures.h>
#include <robocon2022_essentials_msgs/UserAction.h>

#define BT_QUEUE_SIZE 5
#define MAX_DETECTION_DIFF_TIME 1.0 //  1000ms

//	publisher
ros::Publisher user_action_pub_;

//  Body tracker buffer
body_tracker_msgs::BodyTrackerArray last_body_tracker_array;

//  enum
enum class GestureType
{
  GESTURE_WAVING      = 0,
  GESTURE_SWIPE_LEFT  = 1,
  GESTURE_SWIPE_RIGHT = 2,
  GESTURE_SWIPE_UP    = 3,
  GESTURE_SWIPE_DOWN  = 4,
  GESTURE_PUSH        = 5,
};

//  UserStatus including BodyTracker and Gesture
struct UserStatus {
  body_tracker_msgs::BodyTracker body_tracker;
  body_tracker_msgs::Gesture gesture;
  double distance;
};

void callbackBodyTracker(const body_tracker_msgs::BodyTrackerArray& body_trackers)
{
  //  set
  last_body_tracker_array = body_trackers;

  //  debug
  // geometry_msgs::Point32 center = body_trackers.detected_list[0].position2d;
  // std::cout << "Position2D:" << std::endl
  // << "x: " << center.x << std::endl
  // << "y: " << center.y << std::endl
  // << "=========================" << std::endl;
}

void callbackGestures(const body_tracker_msgs::Gestures& gestures)
{
  ROS_INFO("subscribe Gesture"); 
  //  consider condition
  /**
  判断優先度:
  user_idが一致するか
  トラッキングしているユーザーが中央に近い
  confidenceが高い (これはnuitrackの方で上げておけばいいからいらないかも)
  タイムスタンプが近い

  探索:
  1. ジェスチャーとbodyを紐付けてmapに保存
  この際タイムスタンプがMAX_DETECTION_DIFF_TIMEを超えてないか判断
  2. センター付近に近い順にソートする
  **/

  /*
  //  map
  std::vector<UserStatus> status_list;

  //  integrate BodyTracker and Gesture
  for(body_tracker_msgs::Gesture gesture : gestures)
  {
    //  SWIP-UP,DOWNのみ検知
    if (gesture.type == GestureType::GESTURE_SWIPE_UP || gesture.type == GestureType::GESTURE_SWIPE_DOWN)
    {
      body_tracker_msgs::BodyTracker user_body = NULL;
      int user_id = gesture.user_id;
      ros::Time timestamp = gestures.header.stamp;

      //  serarch same Body
      ros::Time tracked_at = last_body_tracker_array.header.stamp;
      if(timestamp.toSec() - tracked_at.toSec() > MAX_DETECTION_DIFF_TIME) 
      {
        //  if timed out, break loop. waiting next callback
        break;
      }

      for (body_tracker_msgs::BodyTracker any_body : last_body_tracker_array)
      {
        //  check if user id match body id
        if (any_body.body_id == user_id) {
          //  TODO  ここをxだけにするのもあり
          double distance = sqrt(pow(any_body.position2d.x, 2.0) + pow(any_body.position2d.y, 2.0));
          //  insert into map
          UserStatus status = {
            any_body,
            gesture,
            distance
          };

          status_list.push_back(status);
        }
      }
    }
  }

  //  sort function
  auto compare_status = [](const UserStatus& status_a_, const UserStatus& status_b_)
  {
    if (status_a_.distance > status_b_.distance) {
      return true;
    }
    else if(status_a_.distance == status_b_.distance) {
      //  compare Gesture type
      if(status_a_.gesture.type != GestureType.GESTURE_SWIPE_DOWN && status_b_.gesture.type == GestureType.GESTURE_SWIPE_DOWN)
      {
        return true;
      }
    }
    return false;
  }
  */

  /*
  1. 中央に近いユーザーをピック => x,y共に0に近い
  2. gestureとユーザーが一致するか確認
  */
  body_tracker_msgs::BodyTracker *best_body = nullptr;
  double min_distance = 100;
  for (body_tracker_msgs::BodyTracker any_body : last_body_tracker_array.detected_list)
  {
    //  TODO  ここをxだけにするのもあり
    double distance = sqrt(pow(any_body.position2d.x, 2.0) + pow(any_body.position2d.y, 2.0));
    if (distance < min_distance)
    {
      //  swap
      best_body = &any_body;
      min_distance = distance;
    }
  }
  //  break if null
  if (best_body == nullptr) {
    ROS_INFO("Skip GestureCallback due to NOT FOUND BODY");
    return;
  }

  body_tracker_msgs::Gesture *best_gesture = nullptr;
  //  Gesture compare
  for(body_tracker_msgs::Gesture gesture : gestures.gestures)
  {
    if(gesture.user_id != best_body->body_id) {
      ROS_INFO("Skip (%d) due to different body", gesture.user_id);
      continue;
    }
    //  SWIP-UP,DOWN only
    if (gesture.type == (int)GestureType::GESTURE_SWIPE_UP || gesture.type == (int)GestureType::GESTURE_SWIPE_DOWN)
    {
      best_gesture = &gesture;
      break;
    }
  }
  //  break if null
  if (best_gesture == nullptr) {
    ROS_INFO("Skip GestureCallback due to NOT FOUND GESTURE");
    return;
  }

  //  publish
  ROS_INFO("Found great gesture");

  robocon2022_essentials_msgs::UserAction user_action;
  user_action.user_id = best_gesture->user_id;
  user_action.action_id = best_gesture->type;
  user_action.position2d = best_body->position2d;
  user_action_pub_.publish(user_action);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "robocon2022_essentials_detection");
  ros::NodeHandle nh;
  
  //  publisher
  user_action_pub_ = nh.advertise<robocon2022_essentials_msgs::UserAction>("/essentials_detection/user_action", 10);
  //  subscriber
  ros::Subscriber body_trackers_sub_ = nh.subscribe("/body_tracker_array/position", 10, callbackBodyTracker);
  ros::Subscriber body_gestures_sub_ = nh.subscribe("/body_tracker/gestures", 10, callbackGestures);

  ros::spin();
  return 0;
}