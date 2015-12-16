#include <ros/ros.h>
#include <vicon_bridge/viconGrabPose.h>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <string>
#include <chrono>

#include <boost/asio.hpp>
#include "boost/bind.hpp"
#include "boost/date_time/posix_time/posix_time_types.hpp"

std::string subject, segment;
std::string service;

const std::string grabber = "grabpose_clent";

const short multicast_port = 30001;
const int max_message_count = 2;				//because my head can never be more than 2m above the ground :)

class sender
{
public:
  sender(boost::asio::io_service& io_service,
      const boost::asio::ip::address& multicast_address, 
      float x, float y, float z, float roll, float pitch, float yaw)
    : endpoint_(multicast_address, multicast_port),
      socket_(io_service, endpoint_.protocol()),
      timer_(io_service), x_(x), y_(y), z_(z), roll_(roll), pitch_(pitch), yaw_(yaw)
  {
    std::ostringstream os;
    os << std::fixed << std::setfill ('0') << std::setprecision (4) << x_ 
    				<< ", " << y_ << ", "<< z_ << ", "<< roll_ << ", "
    				<< pitch_ << ", "<< yaw_ ;
    message_ = os.str();

    socket_.async_send_to(
        boost::asio::buffer(message_), endpoint_,
        boost::bind(&sender::handle_send_to, this,
          boost::asio::placeholders::error));

    ROS_INFO_STREAM("Sent (x,y,z, r, p, y): "<< std::setw('0') << std::fixed << std::setprecision(4)<< message_ <<")");
  }

  void handle_send_to(const boost::system::error_code& error)
  {
    if (!error && x_ > max_message_count)
    {
      timer_.expires_from_now(boost::posix_time::seconds(1));
      timer_.async_wait(
          boost::bind(&sender::handle_timeout, this,
            boost::asio::placeholders::error));
    }
  }

  void handle_timeout(const boost::system::error_code& error)
  {
    if (!error )
    {
      std::ostringstream os;

      os << std::fixed << std::setfill ('0') << std::setprecision (4) << x_ 
    				   << ", " << y_ << ", "<< z_ << ", "<< roll_ << ", "
    				   << pitch_ << ", "<< yaw_ ;

      message_ = os.str();

      ROS_WARN("Message Timed Out. Please look into your send::handle_timeout function");

      socket_.async_send_to(
          boost::asio::buffer(message_), endpoint_,
          boost::bind(&sender::handle_send_to, this,
            boost::asio::placeholders::error));
    }
  }

private:
  boost::asio::ip::udp::endpoint endpoint_;
  boost::asio::ip::udp::socket socket_;
  boost::asio::deadline_timer timer_;
  float x_, y_, z_;
  float roll_, pitch_, yaw_;
  std::string message_;

};

void measuretime()
{


}


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

	boost::asio::io_service io_service;
	std::string multicast_address = "235.255.0.1";

	ros::service::waitForService(service_name.str());

	bool persistent = false;
	const ros::M_string & 	header_values = ros::M_string();											
	ros::ServiceClient vicon_pose =	nc.serviceClient<vicon_bridge::viconGrabPose>(service_name.str(), persistent, header_values);

	//ros::Rate looper(30);			//set to 30Hz

	float x, y, z, roll, pitch, yaw;

	double elapsed;
	size_t frameCount = 0;
	double fps = 0;
	

	do
	{
		vicon_bridge::viconGrabPose srv;
		srv.request.subject_name =  subject; 
		srv.request.segment_name =  segment;
		srv.request.n_measurements = 2;
		std::chrono::time_point<std::chrono::high_resolution_clock> begin, now; 
		begin = std::chrono::high_resolution_clock::now();

		if (vicon_pose.call(srv))
		{
			double start = ros::Time::now().toSec();
			x = srv.response.pose.pose.position.x;
			y = srv.response.pose.pose.position.y;
			z = srv.response.pose.pose.position.z;

			roll 	= srv.response.pose.pose.orientation.x;
			pitch 	= srv.response.pose.pose.orientation.y;
			yaw		= srv.response.pose.pose.orientation.z;

/*			ROS_INFO_STREAM(std::setw('0') << std::fixed << std::setprecision(4) << "Translation: (" << x << ", " << y << 
																													", " << z <<")");
			ROS_INFO_STREAM(std::setw('0') << std::fixed << std::setprecision(4) << "Orientation: (" << roll << ", " << 
																											pitch << ", " << yaw <<")");*/
			sender s(io_service, boost::asio::ip::address::from_string(multicast_address), x, y, z, roll, pitch, yaw);
			double end = ros::Time::now().toSec();

			double diff = end - start ;
			// ROS_INFO_STREAM("elapsed: " << std::setw('0') << std::fixed << std::setprecision(4) << diff*1000 \
											// << " (ms)"<< " fps: " << 1/diff/1000 << "(Hz) ");

			now = std::chrono::high_resolution_clock::now();  
			++frameCount;
			elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - begin).count() / 1000.0;

			if(elapsed >= 1)
			{
			  fps = frameCount / elapsed;
			  ROS_INFO_STREAM("fps: " << fps << "Hz ( " << elapsed / frameCount * 1000.0 << " ms)");
			  begin = now;
			  frameCount = 0;
			}

			io_service.run();

			//looper.sleep();
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
