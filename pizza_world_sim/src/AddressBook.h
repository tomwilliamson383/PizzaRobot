// AddressBook.h
// The address book is a class that utilises an unordered map of human readable address (R1, C1 or P2) and their
// corresponding integer ID (1, 3, 15). The address book will give you the ID for a human readable address or visa versa.
// This allows purely integer communication between the Restaurant and Robot as the order is sent containing the 
// restaurant, customer and number of pizzas. 

#ifndef _ADDRESSBOOK_H
#define _ADDRESSBOOK_H

#include <string>
#include <unordered_map>

class AddressBook
{
    private:
        std::unordered_map<std::string, int> LocationsStringToInt;
        std::unordered_map<int, std::string> LocationsIntToString;
    public:
        AddressBook();
        ~AddressBook();
        // find int ID for given human readable string address (eg: R1)
        int AddressString2Int( std::string AddressString );
        // find the human readable string address (eg: R1) for the int ID
        std::string AddressInt2String( int AddressInt );
};
#endif