#include <ros/ros.h>
#include "std_msgs/String.h"
#include <geometry_msgs/Point.h>

#include <vicon_listener/structs.h>
#include <vicon_listener/pose.hpp>
#include <vicon_listener/connection.hpp>
#include <sstream>

#include <vicon_bridge/Markers.h>

#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/lexical_cast.hpp>
#include <iostream>
#include <vector>
#include <boost/serialization/vector.hpp>


class server
{
public:
  server(boost::asio::io_service& io_service, unsigned short port, pose p)
    : acceptor_(io_service,
        boost::asio::ip::tcp::endpoint(boost::asio::ip::tcp::v4(), port))
  {
    p.xtrans;
    p.ytrans;
    p.ztrans;
    p.roll;
    p.pitch;
    p.yaw;
    posevec_.push_back(p);

    // Start an accept operation for a new connection.
    connection_ptr new_conn(new connection(acceptor_.get_io_service()));
    acceptor_.async_accept(new_conn->socket(),
        boost::bind(&server::handle_accept, this,
          boost::asio::placeholders::error, new_conn));
  }

  /// Handle completion of a accept operation.
  void handle_accept(const boost::system::error_code& e, connection_ptr conn)
  {
    if (!e)
    {
      conn->async_write(posevec_,
          boost::bind(&server::handle_write, this,
            boost::asio::placeholders::error, conn));
    }

    // Start an accept operation for a new connection.
    connection_ptr new_conn(new connection(acceptor_.get_io_service()));
    acceptor_.async_accept(new_conn->socket(),
        boost::bind(&server::handle_accept, this,
          boost::asio::placeholders::error, new_conn));
  }

  /// Handle completion of a write operation.
  void handle_write(const boost::system::error_code& e, connection_ptr conn)
  {
  }

  ~server()
  {
       ROS_WARN("\nServer destructor called.");
  }

private:
  boost::asio::ip::tcp::acceptor acceptor_;
  std::vector<pose> posevec_;
};

int main(int argc, char **argv)
{
/*  ros::init(argc, argv, "RIO_Sender");

  ros::NodeHandle n;
  ros::Publisher sendit = n.advertise<std_msgs::String>("send", 1000);
  ros::Rate loop_rate(10);

  while (ros::ok())
  {
    std_msgs::String msg;
    ss << ""
  }*/
  return 0;
}