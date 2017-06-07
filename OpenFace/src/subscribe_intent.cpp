#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include "OpenFace/intent_msg.h"
#include "OpenFace/intent_msg_all.h"

#include "std_msgs/String.h"



#include <fstream>
#include <sstream>



class subscribe_intent
{

  //subscribe topic
  int c;



public:
  subscribe_intent(ros::NodeHandle nh){
  ros::Subscriber sub = nh.subscribe("results_interaction_intent", 1000, &subscribe_intent::callback, this);
  }
  ~subscribe_intent();
    void callback(const OpenFace::intent_msg_all& msg);
  

};

/*
subscribe_intent::subscribe_intent(int my_c)
{
  c = my_c;
}
*/

subscribe_intent::callback(const OpenFace::intent_msg_all& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
  //std::cout << my_c <<std::endl;
}





void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
    cv::waitKey(30);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "intent_listener");
  ros::NodeHandle nh;
  cv::namedWindow("view");
  cv::startWindowThread();

  //subscribe image
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("camera/image", 1, imageCallback);



  //subscribe intent class
  Listener_intent(nh) subscribe_intent;









  ros::spin();
  cv::destroyWindow("view");
}















/**
 * This tutorial demonstrates subscribing to a topic using a class method as the callback.
 */

// %Tag(CLASS_WITH_DECLARATION)%
class Listener
{
public:
  void callback(const std_msgs::String::ConstPtr& msg);
};
// %EndTag(CLASS_WITH_DECLARATION)%

void Listener::callback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}


