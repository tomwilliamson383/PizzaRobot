// Robot.cpp
// The implementation file for the Robot and its main function for initialising the instance of a Robot
// and running it (allows multiple robots to be easily run from the terminal).

#include <iostream>
#include <unordered_map>
#include <chrono>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/UInt64MultiArray.h"

#include "AddressBook.h"
#include "Robot.h"



Robot::Robot(int ID)
{
  RobotID = ID;
}

Robot::~Robot()
{}

// Callback for the orders subscriber
void Robot::OldestOrderCallback(const std_msgs::UInt64MultiArray::ConstPtr& msg)
{
  class AddressBook MyAddressBook; // put in constructor of robot

  PreviousOrder = CurrentOrder;

  int RestaurantInt = msg->data[0];
  int CustomerInt = msg->data[1];
  int NumPizzas = msg->data[2];
  int OrderNum = msg->data[3];

  std::string RestaurantString;
  std::string DestinationString;
  RestaurantString  = MyAddressBook.AddressInt2String(RestaurantInt);
  DestinationString = MyAddressBook.AddressInt2String(CustomerInt);

  CurrentOrder.Restaurant  = RestaurantString;
  CurrentOrder.Destination = DestinationString;
  CurrentOrder.NumOfPizzas = NumPizzas;
  CurrentOrder.OrderNum    = OrderNum;


  if (CurrentOrder.OrderNum != PreviousOrder.OrderNum)
  {
    Rb.DeliveryService(CurrentOrder.Restaurant, CurrentOrder.Destination);
    std::cout << "Completed - Order Number: " << OrderNum << " Restaurant: " << RestaurantString << " Customer: " << DestinationString << " Number of Pizzas: " << NumPizzas << std::endl;
    Rb.DeliveryService(CurrentOrder.Destination, CurrentOrder.Restaurant);
  }
}

// Subscribe to the orders topic to enable the restaurant to publish its oldest order
void Robot::GetOldRestOrder(ros::NodeHandle n)
{
  ros::Subscriber oldest_order_from_rest_sub = n.subscribe("oldest_order_from_rest", 1000, &Robot::OldestOrderCallback, this);
  ros::Rate loop_rate(10);
  //std::cout << "Order number: " << CurrentOrder.OrderNum << std::endl;
  ros::spin();
  loop_rate.sleep();
}

// Run the robot in a loop between getting the oldest order at restraunts from the orders topic and performing the order
void Robot::RobotRun(int argc, char **argv)
{

  char buffer [50];
  sprintf(buffer, "RobotNode%d", RobotID);
  ros::init(argc, argv, buffer);
  ros::NodeHandle n;
  Rb.Init(n,RobotID);
  GetOldRestOrder(n);
}

// Initialise a robot specified by the terminal UI and run it (loop between receiving and performing orders)
int main(int argc, char **argv)
{
  std::cout << "Please Select A Robot To Initialise: " << std::endl;
  std::cout << "Select [0] for JEFFREY: " << std::endl;
  std::cout << "Select [1] for FENTON:  " << std::endl;

  int NewRobotID;
  std::cin >> NewRobotID;
  char buffer [50];
  sprintf(buffer, "RobotNode%d", NewRobotID);

  ros::init(argc, argv, buffer);
  class Robot MyRobot(NewRobotID);
  MyRobot.RobotRun(argc, argv);
  ros::NodeHandle nh;

  return 0;
}
