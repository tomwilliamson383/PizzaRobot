// Restaurant.h
// The restaurant class contains the core functionality for the restaurant and initialising the publication of its oldest order to 
// the 'oldest\_order\_from\_rest' topic. Restaurant.cpp also contains the main function to initialise an instance of the 
// sRestaurant class and start it running, ready to receive orders from the terminal.

#ifndef _RESTAURANT_H
#define _RESTAURANT_H

#include <string>
#include <queue>
#include <unordered_map>

class Restaurant
{
    private:
        class AddressBook MyAddressBook;
        struct Order {
            std::string Restaurant;
            std::string Destination;
            int NumOfPizzas;
        };
        // FIFO queue for orders
        std::queue<Order> OrderQueue;
        int OrderNumber;
        // Add an order to the order queue from the UI terminal 
        void GetOrder();
        // Publish the provided order information: restaurant, customer destination and the number of pizzas
        void PutOutOldestOrder(ros::Publisher oldest_order_from_rest_pub, int RestaurantInt, int DestinationInt, int UINumPizzas);
    public:
        Restaurant();
        ~Restaurant();
        // Initialise the publisher to the orders topic and only publish the oldest order when a robot is subscribed (ready to take the order)
        void RunRestaurant(int argc, char **argv);
};
#endif
