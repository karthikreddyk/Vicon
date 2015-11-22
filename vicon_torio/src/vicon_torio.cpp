#include <ros/ros.h>
#include "std_msgs/String.h"
#include <iomanip>          //for std::fixed and std::setprecision
#include <ros/console.h>

#include <sstream>
#include <geometry_msgs/Twist.h>

#include <vicon_bridge/Markers.h>

#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/lexical_cast.hpp>
#include <iostream>
#include <vector>
#include <boost/serialization/vector.hpp>

void poseReceiver(const geometry_msgs::Twist::ConstPtr& posemsg)
{
  ROS_INFO_STREAM(std::setprecision(4) << std::fixed 
                << "linear position: (" << posemsg -> linear /*.x << 
                  ", " << posemsg.linear.y << ", " << posemsg.linear.z*/ << ")"
                << "\nangular position: (" << posemsg -> angular/*.x << 
                  ", " << posemsg.angular.y << ", " << posemsg.angular.z*/ <<")");
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "vicon_torio", ros::init_options::AnonymousName);
  ros::NodeHandle n_rio;

  ros::Subscriber posesub = n_rio.subscribe("pose_", 1000, &poseReceiver);

  ros::spin();
  return 0;
}