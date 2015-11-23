#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <vicon_bridge/viconGrabPose.h>
#include <geometry_msgs/Pose.h>
#include <cstdlib>

std::string subject, segment;

std::string M_string()
{
	std::string key = "Successful connection. Ose Patrick!"
	return key;
}

/*    // Service Server
    ROS_INFO("setting up grab_vicon_pose service server ... ");
    m_grab_vicon_pose_service_server = nh_priv.advertiseService("grab_vicon_pose", &ViconReceiver::grabPoseCallback,
                                                                this);*/
    	//This is the service I want to subscribe to. Line 618. vicon_bridge.cpp.

int main(int argc, char** argv)
{
	ros::init(argc, argv, "cpp_listener");

	const std::string& service_name = "grab_vicon_pose";
	bool persistent = false;
	const M_string& header_values = M_string();

	if (argc != 3)
	  {
	    ROS_INFO("usage: rosrun vicon_bridge grabpose <subject_name> <segment_name>");
	    return 1;
	  }

	subject = atoll(argv[1]);
	segment = atoll(argv[2]);

	std::string base_name = "vicon";

	std::string advertname = base_name + "/" + subject + "/" + segment;

	ros::NodeHandle nc;
	ros::service::waitForService(service_name);
	// ros::ServiceClient vicon_pose = 	nc.serviceClient<vicon_bridge::viconGrabPose>(service_name);
	ros::ServiceClient vicon_pose = 	nc.serviceClient(service_name, persistent, \
												header_values)			[inline]
	vicon_bridge::viconGrabPose srv;

	vicon_pose.call(srv);



	srv.request.subject_name =  subject;
	srv.request.segment_name =  segment;
	srv.request.n_measurements = 1000;
	if (vicon_pose.call(srv))
	{
		ROS_INFO_STREAM("Pose: " << srv.response.pose.pose.position.x << ")");
	}
	else
	{
		ROS_ERROR("Failed to call service viconGrabPose");
		return 1;
	}
	return 0;
}
