//Motion Control.h

#ifndef _MOTIONCONTROL_H
#define _MOTIONCONTROL_H


#include <ros/ros.h>
#include <ros/console.h>
#include "odo.h"
#include <iostream>
#include <geometry_msgs/Twist.h>
#include <vector>


class MotionControl
{
  public:
      MotionControl(ros::NodeHandle nh, std::vector<int> v, int ID);    // Constructor
      ~MotionControl();                     // Destructor

      void PublishMotion();                // Publishes velocities of Turtlebot
      void PDcontroller();
      void DecideMotion(double c_I, double c_L);
      void DecideIntersectionDirection();
      bool StartingOrientationCorrection(double Yaw);
      void ConcatPublishString();
      bool CheckGoalStreet(bool result);
      void LocalFrameUpdateOrientation(double Yaw);

  private:
      ros::Publisher pub_vel_;
      ros::NodeHandle _MyNodeHandle;

      int RobotID;

      //values coincide with the change in directions given based on the
      //heading of the robot
      //helps change the global varaible direction into a local direction
      int OrientationMatrix[5][5] = {{0,0,0,0,0},{0,3,4,2,1},{0,1,2,3,4},{0,4,3,1,2},{0,2,1,4,3}};

      char _TwistPubString [50];

      //sample (test) vector of directions
      std::vector<int> vDirections;

      double YellowLineCentroid;
      double RedIntersectionCentroid;

      //turn variables
      int NextTurnDirection;
      bool TurnCompletionFlag;
      bool TurnCommencedFlag;
      int YawBucket;



      //approximately centre
      const int IdealCentroid = 568;
      const int Centroid2VelocityFactor = 1552;

      //speed constants
      double MaxSpeed;
      double TurnSpeed;
      double MaxTurnSpeed;
      double NoSpeed;

      //controller variables
      double RequiredCentroid;
      double ZAngVelocity;
      double XLinVelocity;
      double Kp;
      double Ki;
      double Kd;
      double CurrentTime;
      double PreviousTime;
      double Error;
      double PreviousError;
};

#endif
