// Robot.h
// The robot class contains the core functionality for the Robot, utilises the methods from RobotBrain
// and initialising its subscription to the 'oldest\_order\_from\_rest' topic published from the Restaurant.
// Robot.cpp also contains the main function to initialise an instance of the Robot class and start it running.


#ifndef _ROBOT_H
#define _ROBOT_H

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/UInt64MultiArray.h"
#include "RobotBrain.h"

class RobotBrain;

class Robot
{
    private:
        struct Order {
            std::string Restaurant;
            std::string Destination;
            int NumOfPizzas;
            int OrderNum;
        };
        struct Order CurrentOrder;
        struct Order PreviousOrder;
        bool CarryingOrder;
        class RobotBrain Rb;
        int RobotID;

        // Callback for the orders subscriber
        void OldestOrderCallback(const std_msgs::UInt64MultiArray::ConstPtr& msg);
        // Subscribe to the orders topic to enable the restaurant to publish its oldest order
        void GetOldRestOrder(ros::NodeHandle n);

    public:
        Robot(int ID);
        ~Robot();
        // Run the robot in a loop between getting the oldest order at restraunts from the orders topic and performing the order
        void RobotRun(int argc, char **argv);

};
#endif
