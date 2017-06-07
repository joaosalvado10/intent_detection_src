#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "OpenFace/My_message.h"

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
void GazeCallback(const OpenFace::My_message::ConstPtr& msg)
{
  ROS_INFO("I heard: [%f]", msg->pose_tra_x);
  ROS_INFO("I heard: [%f]", msg->pose_tra_y);
  ROS_INFO("I heard: [%f]", msg->pose_tra_z);
  ROS_INFO("I heard: [%f]", msg->pose_rot_x);
  ROS_INFO("I heard: [%f]", msg->pose_rot_y);
  ROS_INFO("I heard: [%f]", msg->pose_rot_z);

  ROS_INFO("I heard: [%f]", msg->gaze_0_rot_x);
  ROS_INFO("I heard: [%f]", msg->gaze_0_rot_y);
  ROS_INFO("I heard: [%f]", msg->gaze_0_rot_z);
  ROS_INFO("I heard: [%f]", msg->gaze_1_rot_x);
  ROS_INFO("I heard: [%f]", msg->gaze_1_rot_y);
  ROS_INFO("I heard: [%f]", msg->gaze_1_rot_z);
  
  ROS_INFO("I heard: [%ld]", msg->id_model);
  
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "intention_detector");


  ros::NodeHandle n;

 
  ros::Subscriber sub = n.subscribe("pose_gaze", 1000, GazeCallback);


  ros::spin();

  return 0;
}