/// odo.cpp
//
// Implementation file for the Robot Brain,
// This class acts as a mediator for different classes such as LineFollowing,
// BlobDetection, Camera, Odom and Motion Control.
// The robot brain also interfaces with the Robot class in order to take
// commands from the restaurant class
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_broadcaster.h"
#include "RobotBrain.h"
#include "BlobDetection.h"
#include "camera.h"
#include "LineFollowing.h"
#include "IntersectionDetection.h"
#include "MotionControl.h"
#include "A-star-path-planning.h"



//constructor
RobotBrain::RobotBrain()
{

}

//Initialise function to set up the main node handle
void RobotBrain::Init(ros::NodeHandle nh, int ID)
{
  _MyNodeHandle = nh;
  RobotID = ID;
  RunBlobDetection = false;
}

//Destructor
RobotBrain::~RobotBrain()
{

}

//This function creates the classes that are dependant on the Robot Brain for Delivery
// The origin and Destination are filtered by the A* Plannner in order to find the
//most appropriate path for the robot
void RobotBrain::DeliveryService(std::string Origin, std::string Destination)
{

  //create instances of classes needed and the variable where will store
  // the image file
  cv::Mat cv_image;
  LineFollowing lf_(_MyNodeHandle);
  IntersectionDet Id_(_MyNodeHandle);

  ros::Subscriber sub_odom_;
  //set up the Odometry
  odo MyOdom(_MyNodeHandle);

  //tailor the subscribe message for a certain namespace (i.e the specific robot)
  ConcatSubscribeString();
  sub_odom_ = _MyNodeHandle.subscribe(_OdomSub, 100, &odo::OdoCallback, &MyOdom);

  //set up the path finder
  PathFinder MyPathFinder;
  //directions will be stored in a vector for easy deletion of elements
  std::vector<int> Directions;

  //returns a vector of directions
  Directions = MyPathFinder.StartEndDirections(Origin,Destination);

  //need to flip the vector for use within motion control
  //allows for pop_back
  reverse(Directions.begin(), Directions.end());

  //set up motion control, blob detection, camera converters on three sides
  MotionControl Mc_(_MyNodeHandle, Directions, RobotID);

  BlobDetection f_Bd_(_MyNodeHandle,0);
  BlobDetection l_Bd_(_MyNodeHandle,1);
  BlobDetection r_Bd_(_MyNodeHandle,2);

  FrontImageConverter f_ic(&lf_, &Id_, RobotID, &f_Bd_);
  LeftImageConverter l_ic(&lf_, &Id_, RobotID, &l_Bd_);
  RightImageConverter r_ic(&lf_, &Id_, RobotID, &r_Bd_);

  // print path for the tester's knowledge
  for (int i = 0; i < Directions.size(); i++)
  {
    std::cout << Directions[i] << std::endl;
  }

  //set as false for immediate correction upon startup
  bool OrientationFlag = false;

  //create a loop that runs until the goal is satisfied
  while(ros::ok())
  {
    //getter functions that are needed by the brain to service other classes
    double Yaw = MyOdom.GetRobotYaw();
    double c_I = f_ic.GetInterSectionCentroid();
    double c_L = f_ic.GetLineCentroid();

    //This function accesses the Orientation Matrix to decipher if the reference
    //frame needs to be updated
    Mc_.LocalFrameUpdateOrientation(Yaw);

    //This runs on the first iteration of the loop
    if(!OrientationFlag)
    {
      //the function here returns a boolean to the OrientationFlag when the Robot
      //is facing the desired direction on the road.

      OrientationFlag = Mc_.StartingOrientationCorrection(Yaw);
    }

    //On any other iteration above 1, run the robots main driving functionality
    else
    {

      //this will ultimately move the robot to where we need it to be
      Mc_.DecideMotion(c_I, c_L);

      //Goal Checking
      bool LeftResult = l_ic.BlobCheck();
      bool RightResult = r_ic.BlobCheck();
      bool result = false;
      bool goalAcheived = false;

      //if the goal is acheived on either the left or the right camera
      //we can consider the
      if(LeftResult || RightResult)
      {
        result = true;
      }

      //check if the vector is empty, and take the result parameter to Decide
      //course of action in terms of motion

      goalAcheived = Mc_.CheckGoalStreet(result);

      if(goalAcheived == true){
          //exit the loop and ask for another command
          break;
        }



    }
    ros::Rate loop_rate( 10 );
    ros::spinOnce();
    loop_rate.sleep();
  }

}

// set up the ROS message tailored to the robot in question
void RobotBrain::ConcatSubscribeString()
{
  sprintf(_OdomSub, "tb3_%d/odom",RobotID);

}
