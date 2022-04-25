// odo.h
//
// Header file for odometry (finding turtlebots location)

#ifndef _ROBOTBRAIN_H
#define _ROBOTBRAIN_H

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include <geometry_msgs/Twist.h>
#include <iostream>
#include <cstdio>


class RobotBrain
{
    public:
        RobotBrain();    // Constructor
        ~RobotBrain();   // Destructor
        void Init(ros::NodeHandle nh, int ID);
        void DeliveryService(std::string Origin, std::string Destination);
        void ConcatSubscribeString();
        bool OrientationFlag;


    private:
		ros::NodeHandle _MyNodeHandle;
		int RobotID;
    char _OdomSub [50];
    bool RunBlobDetection;

};

#endif
