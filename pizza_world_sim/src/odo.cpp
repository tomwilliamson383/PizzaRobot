/// odo.cpp
//
// Implementation file for the odometer, that can use the position provided by the
// callback to track the turtlebot’s lap number and if it is in the winning region of the maze

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_broadcaster.h"
#include "odo.h"

odo::odo(ros::NodeHandle nh)
{
    _MyNodeHandle = nh;

}

odo::~odo()
{

}

// Odometry callback to update the turtlebot's x and y coordinates
void odo::OdoCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    x = msg->pose.pose.position.x;
    y = msg->pose.pose.position.y;

    Yaw = tf::getYaw(msg->pose.pose.orientation);
    std::cout << "YAW: " << Yaw << std::endl;

}

double odo::GetRobotYaw()
{
  return Yaw;
}
