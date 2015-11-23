#include <ros/ros.h>
#include <vicon_bridge/viconGrabPose.h>
#include <iostream>
#include <iomanip>
#include <sstream>

std::string subject, segment;
std::string service;

const std::string grabber = "grabpose_clent";

int main(int argc, char** argv)
{
	uint32_t options = 0;

	ros::init(argc, argv, grabber, options);

	if (argc != 4)
	  {
	    ROS_INFO_STREAM("\n\n" << 
	    				"                   usage: rosrun vicon_bridge grabpose <subject_name> <segment_name> <service> \n"
	    				<< std::setw(35) << "To subscribe to pose, " << "set <service> to <grab_vicon_pose>\n"	    				
	    				<< std::setw(55) << "To subscribe to calibration segment, " << "set <service> to <calibrate_segment>\n"
	    				<< std::setw(75) << "You could check for available service with rosservice list\n");
	    return 1;
	  }

  	for(size_t i = 1; i < (size_t)argc; ++i)
  	{
    	
    	subject = argv[1];
		segment = argv[2];
		service = argv[3];	
	}

	std::string base_name = "vicon";

	std::string advertname = base_name + "/" + service ;

	ROS_INFO_STREAM("Subscribing to: /" << base_name <<  "/" << service << "\t on topic: " 
					<< base_name <<  "/" << subject  << "/" << segment);

	ros::NodeHandle nc;

	std::ostringstream service_name;
	service_name << base_name << "/" << service;

	ros::service::waitForService(service_name.str());

	bool persistent = false;
	const ros::M_string & 	header_values = ros::M_string();											
	ros::ServiceClient vicon_pose =	nc.serviceClient<vicon_bridge::viconGrabPose>(service_name.str(), persistent, header_values);

	ros::Rate looper(100);			//set to 30Hz

	float x, y, z, roll, pitch, yaw;

	vicon_bridge::viconGrabPose srv;
	do
	{
		srv.request.subject_name =  subject; 
		srv.request.segment_name =  segment;
		srv.request.n_measurements = 2;
		if (vicon_pose.call(srv))
		{
			x = srv.response.pose.pose.position.x;
			y = srv.response.pose.pose.position.y;
			z = srv.response.pose.pose.position.z;

			roll 	= srv.response.pose.pose.orientation.x;
			pitch 	= srv.response.pose.pose.orientation.y;
			yaw		= srv.response.pose.pose.orientation.z;

			ROS_INFO_STREAM(std::setw('0') << std::fixed << std::setprecision(4) << "Translation: (" << x << ", " << y << ", " << z <<")");

			ROS_INFO_STREAM(std::setw('0') << std::fixed << std::setprecision(4) << "Orientation: (" << roll << ", " << pitch << ", " << yaw <<")");

			looper.sleep();
		}
		else
		{
			ROS_ERROR_STREAM("Failed to call service /" <<  service_name.str());
			return 1;
		}
	}
	while(ros::ok());
	return 0;
}
