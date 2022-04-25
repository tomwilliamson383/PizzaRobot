// A-star-path-planning.h

// The Path Finder is provided a start and end node and uses the A* algorithm to find the shortest path between them.
// The sequence of nodes is then translated into direction commands that the Robot can check for each intersection.
// This class contains open source code that is referenced in A-star-path-planning.cpp

#ifndef _A_STAR_PATH_PLANNING_H
#define _A_STAR_PATH_PLANNING_H

#include <iostream>
#include <string>
#include <algorithm>
#include <iostream>
#include <vector>
#include <list>
#include <thread>
#include <atomic>
#include <condition_variable>
#include <math.h>
using namespace std;

class PathFinder
{
    public:
        PathFinder();
        ~PathFinder();
        // The public function: outputs the sequence of global directions along the shortest path between the input start and end node
        std::vector<int> StartEndDirections(std::string StartNode, std::string EndNode);

    private:

        struct sNode
        {
            bool bObstacle = false;			// Is the node an obstruction?
            bool bSteppingStone = false;	// Is the node a stepping stone (not printed in final path list)
            bool bCustomer = false;
            bool bIntersection = false;
            bool bChargingStation = false;
            bool bRestaurant = false;
            bool bVisited = false;			// Have we searched this node before?
            float fGlobalGoal;				// Distance to goal so far
            float fLocalGoal;				// Distance to goal if we took the alternative route
            int x;							// Nodes position in 2D space
            int y;
            std::string NodeName;
            vector<sNode*> vecNeighbours;	// Connections to neighbours
            sNode* parent;					// Node connecting to this node that offers shortest parent
        };

        sNode *nodes = nullptr;
        int nMapWidth = 7;
        int nMapHeight = 12;

        sNode *nodeStart = nullptr;
        sNode *nodeEnd = nullptr;

        vector <std::string> NodesPath;
		vector <std::string> DestinationNodesPath;
        vector <sNode> IntersectionStartEndNodesPath;
        std::vector<int> Directions;

        int SteppingStoneCoords[15][2] = {{0,1},{0,2},{6,2},{0,3},{4,3},{2,4},{3,4},{5,4},{6,5},{0,6},{6,6},{2,7},{3,7},{5,7},{0,8}};

        int CustomerCoords[12][2] = {{0,0},{6,0},{3,1},{5,1},{4,2},{6,3},{1,4},{0,5},{4,7},{1,9},{3,9},{2,10}};
        std::string CustomerNames[12] = {"C1","C2","C3","C4","C5","C6","C7","C8","C9","C10","C11","C12"};

        int IntersectionCoords[10][2] = {{2,1},{4,1},{6,1},{0,4},{4,4},{6,4},{0,7},{6,7},{0,9},{2,9}};
        std::string IntersectionNames[10] = {"I1","I2","I3","I4","I5","I6","I7","I8","I9","I10"};

        int ChargingStationCoords[2][2] = {{2,2},{1,11}};
        std::string ChargingStationNames[2] = {"P1","P2"};

        int RestaurantCoords[2][2] = {{4,5},{1,7}};
        std::string RestaurantNames[2] = {"R1","R2"};

        // Initialise the start and end nodes provided
        virtual bool OnUserCreate(std::string StartNode, std::string EndNode);
        // Initialise the map of nodes
        void Init_Map();
        // Perform A* to find the shortest path of nodes between the start and end node
        int Solve_AStar();
        // Outputs the sequence of nodes along the shortest path between the start and end node
        std::vector<std::string> GivePath(std::string StartNode, std::string EndNode);
        // Covert the sequence of nodes into a sequence of directions in the global reference frame
        int DirectionsFromPath();
};

#endif
