// odo.h
//
// Header file for odometry (finding turtlebots location)

#ifndef _ODOMETRY_H
#define _ODOMETRY_H

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include <geometry_msgs/Twist.h>
#include <iostream>


class odo
{
    public:
        odo(ros::NodeHandle nh);    // Constructor
        ~odo();   // Destructor
		void OdoCallback( const nav_msgs::Odometry::ConstPtr& msg );
    double GetRobotYaw();

    private:
		ros::NodeHandle _MyNodeHandle;
		float x = 0;
		float y = 0;
    double Yaw;


};

#endif
