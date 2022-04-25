// AddressBook.cpp
// Implementation file for the Address Book with methods to provide an address for an ID or visa versa

#include <string> 
#include <iostream>
#include "AddressBook.h"

AddressBook::AddressBook()
{
  // Create a reference for incoming ints to strings (turn into method or own class)
  // Convert Restraunt and Customer to int (make into method)
  // The map for Restaunts and Customers to a reference number (to send over the message)
  int NumOfCustomers = 12;
  int NumOfRestaurants = 2;
  std::string CustomerNames[NumOfCustomers] = {"C1","C2","C3","C4","C5","C6","C7","C8","C9","C10","C11","C12"};
  std::string RestaurantNames[NumOfRestaurants] = {"R1","R2"};

  int IndexCounter = 0;
  // Run through customers assigning a number to each
  for (int i = 0; i < NumOfCustomers; i++)
  {
    LocationsStringToInt[ CustomerNames[i] ] = IndexCounter;
    LocationsIntToString[ IndexCounter ] = CustomerNames[i];
    IndexCounter++;
  }
  // Run through restaurants assigning a number to each
  for (int i = 0; i < NumOfRestaurants; i++)
  {
    LocationsStringToInt[ RestaurantNames[i] ] = IndexCounter;
    LocationsIntToString[ IndexCounter ] = RestaurantNames[i];
    IndexCounter++;
  }
}

AddressBook::~AddressBook()
{}

int AddressBook::AddressString2Int( std::string AddressString )
{
  // find int ID for given human readable string address (eg: R1)
  std::unordered_map<std::string,int>::iterator MapIterator;
  MapIterator = LocationsStringToInt.find( AddressString );
  int AddressInt = (*MapIterator).second;
  return AddressInt;
}

std::string AddressBook::AddressInt2String( int AddressInt )
{
  // find the human readable string address (eg: R1) for the int ID
  std::unordered_map<int,std::string>::iterator MapIterator;
  MapIterator = LocationsIntToString.find( AddressInt );
  std::string AddressString = (*MapIterator).second;
  return AddressString;
}