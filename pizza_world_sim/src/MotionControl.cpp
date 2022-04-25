#include <vector>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "MotionControl.h"
#include "odo.h"
#include <cmath>

MotionControl::MotionControl( ros::NodeHandle nh, std::vector<int> v, int ID) // Constructor
: _MyNodeHandle(nh)
{
  ros::Rate loop_rate( 10 );
  //set as
  RobotID = ID;
  YellowLineCentroid = 568;
  CurrentTime = ros::Time::now().toSec();
  PreviousTime = 0.0;
  Error = 0.0;
  PreviousError = 0.0;
  Kp = 4.5;
  Kd = 0.5;
  ZAngVelocity = 0.0;
  XLinVelocity = 0.1;
  MaxSpeed = 0.22;
  TurnSpeed = 0.6;
  MaxTurnSpeed = 2.84;
  NoSpeed = 0.0;
  TurnCompletionFlag = false;
  TurnCommencedFlag = false;
  vDirections = v;
  YawBucket = 1;

}

MotionControl::~MotionControl() // Destructor
{

}

//the purpose of this function is to re-orientate the robot until it is facing
//the right way. The boolean value confirms that is should not be run again.
bool MotionControl::StartingOrientationCorrection(double Yaw)
{
  //turn the robot on the spot
  bool result = false;
  XLinVelocity = NoSpeed;

  //read the first direction, this is the direction that tells the robots
  //the initial direciton that it should be travling in

  int StartingDirection = vDirections.back();
  std::cout <<"Starting Direction" << StartingDirection << std::endl;


  //if the robot is to go up on the first direction
  if(StartingDirection == 1)
  {
    //check against a tolerance
    if(abs(M_PI/2 - Yaw) <= 0.4)
    {
      result = true;
    }
    else
    {
      ZAngVelocity = MaxTurnSpeed/3;
      PublishMotion();
    }
  }

  //if the robot is to go down on the first direction
  else if(StartingDirection == 2)
  {
    //check against a tolerance
    if(abs(M_PI/2 - Yaw) <= 0.4)
    {
      result = true;
    }
    else
    {
      ZAngVelocity = -MaxTurnSpeed/3;
      PublishMotion();
    }
  }

  //if the robot is to go right on the first direction
  else if(StartingDirection == 4)
  {
    if(abs(0.0 - Yaw) <= 0.4)
    {
      result = true;
    }
    else
    {
      ZAngVelocity = MaxTurnSpeed/3;
      PublishMotion();
    }
  }

  //if the robot is to go left on the first direction
  else if(StartingDirection == 3)
  {

    if(abs(M_PI - Yaw) <= 0.4)
    {
      result = true;

    }
    else
    {
      ZAngVelocity = MaxTurnSpeed/3;
      PublishMotion();
    }
  }

  //pop off this first direction, to allow for the intersection direction
  if(result == true)
  {
    vDirections.pop_back();
  }

  return result;
}

// provides the main functionality
void MotionControl::DecideMotion(double c_I, double c_L)
{

  YellowLineCentroid = c_L;
  RedIntersectionCentroid = c_I;
  std::cout<<"INTER"<<RedIntersectionCentroid<<std::endl;
  std::cout<<"LINE"<<YellowLineCentroid<<std::endl;

  //move to the next direction if a previous intersection has been traversed
  if(TurnCompletionFlag)
  {
    //delete the direction just completed
    vDirections.pop_back();
    //reset the flag
    TurnCompletionFlag = false;
  }

  //retrieve the next direction
  NextTurnDirection = vDirections.back();
  std::cout << "Upcoming Turn: " << NextTurnDirection << std::endl;


  //if an intersection is reached, look to change directions
  if(RedIntersectionCentroid >= 500 && RedIntersectionCentroid <= 650)
  {
    //change direction of the turtlbot upon reaching an intersection

    DecideIntersectionDirection();
    PublishMotion();

    //set a flag to indicate a turn has been commenced
    TurnCommencedFlag = true;

  }

  //as the end of the turn is nearing - look to complete the turn by returning
  //to line following
  else if (RedIntersectionCentroid >= 500 && RedIntersectionCentroid <= 800)
  {
    //constant fowards velocity
    XLinVelocity = MaxSpeed;
    YellowLineCentroid = 300;
    //decide the angular velocity based on line centroid information
    PDcontroller();
    PublishMotion();

  }

  else
  {
    //if a turn has been completed and we are now returning to line following
    if(TurnCommencedFlag){
      //set true to update the next instruction
      TurnCompletionFlag = true;
      //reset for next intersection
      TurnCommencedFlag = false;
    }
    //constant fowards velocity
    XLinVelocity = 0.15;
    //decide the angular velocity based on line centroid information
    PDcontroller();
    PublishMotion();
  }



}

//this function updates the velocity of the robot depending on where it needs
//to go

void MotionControl::DecideIntersectionDirection()
{
  //Consult the OrientationMatrix for local directions
  NextTurnDirection = OrientationMatrix[YawBucket][NextTurnDirection];

  //Go straight (local coordaintes)
  if(NextTurnDirection == 1)
  {
    XLinVelocity = MaxSpeed;
    ZAngVelocity = 0.0;

  }
  //Go Left (local coordintes)
  else if(NextTurnDirection == 3)
  {
    XLinVelocity = MaxSpeed/1.5;
    ZAngVelocity = TurnSpeed;
  }

  //Go Right (Local Coordinates)
  else if(NextTurnDirection == 4)
  {
    XLinVelocity = MaxSpeed/1.5;
    ZAngVelocity = -TurnSpeed;
  }

  //turn around (local coordinates)
  else if(NextTurnDirection == 2)
  {
    //turn on the spot
    XLinVelocity = 0.0;
    ZAngVelocity = MaxTurnSpeed;
  }


}

//this function returns a boolean when the restaurant or house has been found
bool MotionControl::CheckGoalStreet(bool result)
{

  bool Completion = false;

  //if there are no more directions left to make
  if(vDirections.empty())
  {
    std::cout << "The PizzaBot is on the final Street" << std::endl;
    if(result == true)
    {
      //stop the bot to deliver the pizza
      XLinVelocity = NoSpeed;
      ZAngVelocity = NoSpeed;
      PublishMotion();

      //tell the user
      std::cout << "The pizza has been delivered" << std::endl;
      Completion = true;
    }

  }

  return Completion;

}

// this function publishes the twist message and also conducts spin once
void MotionControl::PublishMotion()
{
  ConcatPublishString();
  pub_vel_   = _MyNodeHandle.advertise<geometry_msgs::Twist>( _TwistPubString, 10 );

  // publish the velocity
  geometry_msgs::Twist msg;

  //update the linear velocity about x and the angular velocity about z
  msg.linear.x = XLinVelocity;
  msg.angular.z = ZAngVelocity;

  pub_vel_.publish(msg);
  ros::spinOnce();

}

//splits the available yaw data into 4 sects of information
//these areas help inform the robot of its position when approaching a turn
void MotionControl::LocalFrameUpdateOrientation(double Yaw)
{

  //check against a tolerance
  //facing right on the global frame
  if(Yaw < M_PI/4 && Yaw > -M_PI/4)
  {
    YawBucket = 1;
  }
  //facing up on the global frame
  else if((Yaw < (3*M_PI)/4) && Yaw > M_PI/4)
  {
    YawBucket = 2;
  }
  //facing down on the global frame
  else if((Yaw < -(M_PI)/4) && Yaw > -(3*M_PI)/4)
  {
    YawBucket = 3;
  }
  //fracing left on the global frame
  else if(((Yaw < M_PI && Yaw > (3*M_PI)/4)) || ((Yaw < -(3*M_PI)/4) && Yaw > -M_PI))
  {
    YawBucket = 4;
  }

}

//This controller ultimately finds the line and then stays there
void MotionControl::PDcontroller()
{

  //record current time for Derivative Controller
  CurrentTime = ros::Time::now().toSec();
  //the error is going to be the distance from the middle centroid that we want
  //the turtlebot to maintain
  Error = IdealCentroid - YellowLineCentroid;

  //implement a PD controller
  RequiredCentroid = (Kp * Error) + (Kd * (Error - PreviousError)/(CurrentTime-PreviousTime));

  //convert to angular velocity
  //equation is derived from when the centroid is on the far left of the vision
  //when the centroid approximately middle
  //when then centroid is on the far right of the vision

  ZAngVelocity =  (RequiredCentroid)/(Centroid2VelocityFactor);

  //update iteration values of error and time for the next loop
  PreviousError = Error;
  PreviousTime = CurrentTime;
}

//this function tailors the twist message to the specified robot
void MotionControl::ConcatPublishString()
{
  sprintf(_TwistPubString, "tb3_%d/cmd_vel",RobotID);
}
