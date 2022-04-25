// Restaurant.cpp
// Implementation file for the Restaurant class and contains the main function for initialising the instance of a Restaurant
// and running it (allows multiple restaurants to be easily run from the terminal).

#include <iostream>
#include <string>
#include <unordered_map>
#include "ros/ros.h"
#include "std_msgs/UInt64MultiArray.h"

#include "AddressBook.h"
#include "Restaurant.h"

Restaurant::Restaurant()
{
}

Restaurant::~Restaurant()
{}

// Initialise the publisher to the orders topic and only publish the oldest order when a robot is subscribed (ready to take the order)
void Restaurant::RunRestaurant(int argc, char **argv)
{
  // Start publisher node and send the command
  ros::init(argc, argv, "Restaurant");
  ros::NodeHandle n;
  ros::Publisher oldest_order_from_rest_pub = n.advertise<std_msgs::UInt64MultiArray>("oldest_order_from_rest", 1000);
  ros::Rate loop_rate(10);

  OrderNumber = 1;
  while(1)
  {
    // only publish the order when a robot is subscribed 
    if ((oldest_order_from_rest_pub.getNumSubscribers() >= 1)&&(OrderQueue.size() > 0))
    {
      struct Order LastOrder;
      LastOrder = OrderQueue.front(); // FIFO, publish the oldest order (to ensure customer's a delivered to in order)
      OrderQueue.pop();

      int RestaurantInt;
      int DestinationInt;
      // Convert the restaurant and destination name into their int ID's for communication
      RestaurantInt  = MyAddressBook.AddressString2Int( LastOrder.Restaurant );
      DestinationInt = MyAddressBook.AddressString2Int( LastOrder.Destination );

      PutOutOldestOrder(oldest_order_from_rest_pub, RestaurantInt, DestinationInt, LastOrder.NumOfPizzas);
      std::cout << "Order " << OrderNumber << " sent to Robot." << std::endl;
      OrderNumber++;
    }
    GetOrder();
  }
  ros::spinOnce();
  loop_rate.sleep();
}

// Publish the provided order information: restaurant, customer destination and the number of pizzas
void Restaurant::PutOutOldestOrder(ros::Publisher oldest_order_from_rest_pub, int RestaurantInt, int DestinationInt, int UINumPizzas)
{
  ros::Rate loop_rate(10);
  std_msgs::UInt64MultiArray msg;
  msg.data = {RestaurantInt,DestinationInt,UINumPizzas,OrderNumber};
  oldest_order_from_rest_pub.publish(msg);
}

// Add an order to the order queue from the UI terminal 
void Restaurant::GetOrder()
{
  // Get User Input (make into method)
  std::string UIRestaurantString ;
  std::string UICustomerString;
  int UINumPizzas;

  std::cout << "Restaurant: " << std::endl;
  std::cin >> UIRestaurantString;
  std::cout << "Customer: " << std::endl;
  std::cin >> UICustomerString;
  std::cout << "Number of Pizzas: " << std::endl;
  std::cin >> UINumPizzas;

  struct Order NewOrder;
  NewOrder.Restaurant = UIRestaurantString;
  NewOrder.Destination = UICustomerString;
  NewOrder.NumOfPizzas = UINumPizzas;

  OrderQueue.push(NewOrder);
}

// Initialise a restaurant and run it, it will then receive orders from the terminal and publish the oldest one to robots when they are ready
int main(int argc, char **argv)
{
  class Restaurant MyRestaurant;
  MyRestaurant.RunRestaurant(argc, argv);
  return 0;
}
