#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <vicon_bridge/viconGrabPose.h>
#include <geometry_msgs/Pose.h>
#include <ros/service_client.h>
#include "ros/connection.h"
#include "ros/service_manager.h"
#include "ros/service.h"
#include <ros/datatypes.h>
#include <ros/spinner.h>
#include <sstream>
#include <cstdlib>
#include <stdio.h>

std::string subject, segment;
const std::string grabber = "grabpose_clent";

// typedef std::map<std::string,std::string>ros::M_string;

/*    // Service Server
    ROS_INFO("setting up grab_vicon_pose service server ... ");
    m_grab_vicon_pose_service_server = nh_priv.advertiseService("grab_vicon_pose", &ViconReceiver::grabPoseCallback,
                                                                this);*/
    	//This is the service I want to subscribe to. Line 618. vicon_bridge.cpp.

int main(int argc, char** argv)
{
	uint32_t options = 0;

	ros::init(argc, argv, grabber, options);

	if (argc != 3)
	  {
	    ROS_INFO("usage: rosrun vicon_bridge grabpose <subject_name> <segment_name>");
	    return 1;
	  }

  	for(size_t i = 1; i < (size_t)argc; ++i)
  	{
    	std::string param(argv[i]);

		subject = argv[1];
		segment = argv[2];
	}

	std::string base_name = "vicon";

	std::string advertname = base_name + "/" + subject + "/" + segment;

	ROS_INFO_STREAM("base_name" << "/subject" << "/segment: \t" << base_name <<  "/" << subject  << "/" << segment);

	ros::NodeHandle nc;

	std::ostringstream service_name;
	service_name << base_name <<  "/" << subject  << "/" << segment;

	ros::service::waitForService(service_name.str());

	//ros::ServiceClient vicon_pose = 	nc.serviceClient<vicon_bridge::viconGrabPose>(service_name.str());		//Does not sunscribe to service
/*		ServiceClient ros::NodeHandle::serviceClient	(	const std::string & 	service_name,
															bool 	persistent = false,
															const M_string & 	header_values = M_string() 
															)		[inline]*/	
	bool persistent = false;
	const ros::M_string & 	header_values = ros::M_string();															
	ros::ServiceClient vicon_pose =	nc.serviceClient<vicon_bridge::viconGrabPose>(service_name.str());

	ros::Rate looper(10);	

	vicon_bridge::viconGrabPose srv;
	do
	{
		srv.request.subject_name =  subject; 
		srv.request.segment_name =  segment;
		srv.request.n_measurements = 1000;
		if (vicon_pose.call(srv))
		{
			ROS_INFO_STREAM("Pose: " << srv.response << ")");
		}
		else
		{
			ROS_ERROR("Failed to call service viconGrabPose");
			return 1;
		}
	}
	while(ros::ok());
	return 0;
}
