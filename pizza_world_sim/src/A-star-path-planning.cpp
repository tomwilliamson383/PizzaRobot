// A-star-path-planning.cpp
// Implementation file for the A* path planning. 
// The public function outputs a sequence of global directions when given a start and end node.

// This class uses open source code with full permission from:
// Javidx9 (OneLoneCoder), PathFinding A*, Github Repository (and Youtube Video)
// Last Updated: 08/10/2017
// https://github.com/OneLoneCoder/videos/blob/master/OneLoneCoder_PathFinding_AStar.cpp
// https://youtu.be/icZj67PTFhc
// License details:
// https://github.com/OneLoneCoder/videos/blob/master/LICENSE
// Additional links:
// https://www.github.com/onelonecoder
// https://www.onelonecoder.com
// https://www.youtube.com/javidx9

// Uncomment/comment sd::cout from line 374 to toggle hman readable directions

#include "A-star-path-planning.h"

PathFinder::PathFinder()
{}

PathFinder::~PathFinder()
{}

// Initialise the start and end nodes provided
bool PathFinder::OnUserCreate(std::string StartNode, std::string EndNode)
{
	// Create a 2D array of nodes
	Init_Map();

	int StartX;
	int StartY;
	int EndX;
	int EndY;
	std::string UIStart = StartNode;
	std::string UIEnd = EndNode;

	// Manually position the start and end markers so they are not nullptr
	// std::cout << "Start node: " << std::endl;
	// std::cin >> UIStart;
	// std::cout << "End node: " << std::endl;
	// std::cin >> UIEnd;

	// Search for navigation graph coordinates of start and end node
	// Search if customer
	for (int i = 0; i < 12; i++)
	{
		if (UIStart == CustomerNames[i])
		{
			StartX = CustomerCoords[i][0];
			StartY = CustomerCoords[i][1];
		}
		if (UIEnd == CustomerNames[i])
		{
			EndX = CustomerCoords[i][0];
			EndY = CustomerCoords[i][1];
		}
	}
	// Search if intersection
	for (int i = 0; i < 10; i++)
	{
		if (UIStart == IntersectionNames[i])
		{
			StartX = IntersectionCoords[i][0];
			StartY = IntersectionCoords[i][1];
		}
		if (UIEnd == IntersectionNames[i])
		{
			EndX = IntersectionCoords[i][0];
			EndY = IntersectionCoords[i][1];
		}
	}
	// Search if charging station
	for (int i = 0; i < 2; i++)
	{
		if (UIStart == ChargingStationNames[i])
		{
			StartX = ChargingStationCoords[i][0];
			StartY = ChargingStationCoords[i][1];
		}
		if (UIEnd == ChargingStationNames[i])
		{
			EndX = ChargingStationCoords[i][0];
			EndY = ChargingStationCoords[i][1];
		}
	}
	// Search if restaurant
	for (int i = 0; i < 2; i++)
	{
		if (UIStart == RestaurantNames[i])
		{
			StartX = RestaurantCoords[i][0];
			StartY = RestaurantCoords[i][1];
		}
		if (UIEnd == RestaurantNames[i])
		{
			EndX = RestaurantCoords[i][0];
			EndY = RestaurantCoords[i][1];
		}
	}
	int StartIndex = (StartY * nMapWidth) + StartX;
	int EndIndex = (EndY * nMapWidth) + EndX;
	nodeStart = &nodes[StartIndex];
	nodeEnd = &nodes[EndIndex];
	//std::cout << "Staring node at x:" << nodes[StartIndex].x << " y:" << nodes[StartIndex].y << std::endl;
	//std::cout << "End node at x:" << nodes[EndIndex].x << " y:" << nodes[EndIndex].y << std::endl;
	return true;
}

// Initialise the map of nodes
void PathFinder::Init_Map()
{
	nodes = new sNode[nMapWidth * nMapHeight];

	// Default all nodes to be obstacles
	for (int x = 0; x < nMapWidth; x++)
		for (int y = 0; y < nMapHeight; y++)
		{
			nodes[y * nMapWidth + x].x = x; 
			nodes[y * nMapWidth + x].y = y;
			nodes[y * nMapWidth + x].bObstacle = true;
			nodes[y * nMapWidth + x].bSteppingStone = false;
			nodes[y * nMapWidth + x].parent = nullptr;
			nodes[y * nMapWidth + x].bVisited = false;
			nodes[y * nMapWidth + x].bCustomer = false;
			nodes[y * nMapWidth + x].bIntersection = false;
			nodes[y * nMapWidth + x].bChargingStation = false;
			nodes[y * nMapWidth + x].bRestaurant = false;
			nodes[y * nMapWidth + x].NodeName = "Obstacle";
		}

	// Set up stepping stones
	for (int i = 0; i < 15; i++)
		{
			int x = SteppingStoneCoords[i][0];
			int y = SteppingStoneCoords[i][1];
			nodes[y * nMapWidth + x].bObstacle = false;
			nodes[y * nMapWidth + x].bSteppingStone = true;
			nodes[y * nMapWidth + x].NodeName = "Stepping Stone";
		}

	// Set up targets:
	// Set up customers
	for (int i = 0; i < 12; i++)
		{
			int x = CustomerCoords[i][0];
			int y = CustomerCoords[i][1];
			nodes[y * nMapWidth + x].bObstacle = false;
			nodes[y * nMapWidth + x].bCustomer = true;
			nodes[y * nMapWidth + x].NodeName = CustomerNames[i];
		}
	// Set up intersections
	for (int i = 0; i < 10; i++)
		{
			int x = IntersectionCoords[i][0];
			int y = IntersectionCoords[i][1];
			nodes[y * nMapWidth + x].bObstacle = false;
			nodes[y * nMapWidth + x].bIntersection = true;
			nodes[y * nMapWidth + x].NodeName = IntersectionNames[i];
		}
	// Set up charging stations
	for (int i = 0; i < 2; i++)
		{
			int x = ChargingStationCoords[i][0];
			int y = ChargingStationCoords[i][1];
			nodes[y * nMapWidth + x].bObstacle = false;
			nodes[y * nMapWidth + x].bChargingStation = true;
			nodes[y * nMapWidth + x].NodeName = ChargingStationNames[i];
		}
	// Set up restaurants
	for (int i = 0; i < 2; i++)
		{
			int x = RestaurantCoords[i][0];
			int y = RestaurantCoords[i][1];
			nodes[y * nMapWidth + x].bObstacle = false;
			nodes[y * nMapWidth + x].bRestaurant = true;
			nodes[y * nMapWidth + x].NodeName = RestaurantNames[i];
		}
	// Create connections - in this case nodes are on a regular grid
	for (int x = 0; x < nMapWidth; x++)
		for (int y = 0; y < nMapHeight; y++)
		{
			if(y>0)
				nodes[y*nMapWidth + x].vecNeighbours.push_back(&nodes[(y - 1) * nMapWidth + (x + 0)]);
			if(y<nMapHeight-1)
				nodes[y*nMapWidth + x].vecNeighbours.push_back(&nodes[(y + 1) * nMapWidth + (x + 0)]);
			if (x>0)
				nodes[y*nMapWidth + x].vecNeighbours.push_back(&nodes[(y + 0) * nMapWidth + (x - 1)]);
			if(x<nMapWidth-1)
				nodes[y*nMapWidth + x].vecNeighbours.push_back(&nodes[(y + 0) * nMapWidth + (x + 1)]);
		}
}

// Perform A* to find the shortest path of nodes between the start and end node
int PathFinder::Solve_AStar()
{
	int LARGE = 1000000; // results in avoidance unless the node is reset 
	// Reset Navigation Graph - default all node states
	for (int x = 0; x < nMapWidth; x++)
		for (int y = 0; y < nMapHeight; y++)
		{
			nodes[y*nMapWidth + x].bVisited = false;
			nodes[y*nMapWidth + x].fGlobalGoal = LARGE;
			nodes[y*nMapWidth + x].fLocalGoal = LARGE;
			nodes[y*nMapWidth + x].parent = nullptr;	// No parents
		}

	auto distance = [](sNode* a, sNode* b) // For convenience
	{
		return sqrtf((a->x - b->x)*(a->x - b->x) + (a->y - b->y)*(a->y - b->y));
	};

	auto heuristic = [distance](sNode* a, sNode* b) // So we can experiment with heuristic
	{
		return distance(a, b);
	};

	// Setup starting conditions
	sNode *nodeCurrent = nodeStart;
	nodeStart->fLocalGoal = 0.0f;
	nodeStart->fGlobalGoal = heuristic(nodeStart, nodeEnd);

	// Add start node to not tested list - this will ensure it gets tested.
	// As the algorithm progresses, newly discovered nodes get added to this
	// list, and will themselves be tested later
	list<sNode*> listNotTestedNodes;
	listNotTestedNodes.push_back(nodeStart);

	// if the not tested list contains nodes, there may be better paths
	// which have not yet been explored. However, we will also stop
	// searching when we reach the target - there may well be better
	// paths but this one will do - it wont be the longest.
	while (!listNotTestedNodes.empty() && nodeCurrent != nodeEnd)// Find absolutely shortest path
	{
		// Sort Untested nodes by global goal, so lowest is first
		listNotTestedNodes.sort([](const sNode* lhs, const sNode* rhs){ return lhs->fGlobalGoal < rhs->fGlobalGoal; } );

		// Front of listNotTestedNodes is potentially the lowest distance node. Our
		// list may also contain nodes that have been visited, so ditch these...
		while(!listNotTestedNodes.empty() && listNotTestedNodes.front()->bVisited)
			listNotTestedNodes.pop_front();

		// ...or abort because there are no valid nodes left to test
		if (listNotTestedNodes.empty())
			break;

		nodeCurrent = listNotTestedNodes.front();
		nodeCurrent->bVisited = true; // We only explore a node once


		// Check each of this node's neighbours...
		for (auto nodeNeighbour : nodeCurrent->vecNeighbours)
		{
			// ... and only if the neighbour is not visited and is
			// not an obstacle, add it to NotTested List
			if (!nodeNeighbour->bVisited && nodeNeighbour->bObstacle == 0)
				listNotTestedNodes.push_back(nodeNeighbour);

			// Calculate the neighbours potential lowest parent distance
			float fPossiblyLowerGoal = nodeCurrent->fLocalGoal + distance(nodeCurrent, nodeNeighbour);

			// If choosing to path through this node is a lower distance than what
			// the neighbour currently has set, update the neighbour to use this node
			// as the path source, and set its distance scores as necessary
			if (fPossiblyLowerGoal < nodeNeighbour->fLocalGoal)
			{
				nodeNeighbour->parent = nodeCurrent;
				nodeNeighbour->fLocalGoal = fPossiblyLowerGoal;

				// The best path length to the neighbour being tested has changed, so
				// update the neighbour's score. The heuristic is used to globally bias
				// the path algorithm, so it knows if its getting better or worse. At some
				// point the algo will realise this path is worse and abandon it, and then go
				// and search along the next best path.
				nodeNeighbour->fGlobalGoal = nodeNeighbour->fLocalGoal + heuristic(nodeNeighbour, nodeEnd);
			}
		}
	}
	// Draw Path by starting at the end, and following the parent node trail
	// back to the start - the start node will not have a parent path to follow
	if (nodeEnd != nullptr)
	{
		sNode *p = nodeEnd;

		// Create a vector of all nodes along the path and a vector onl of the star node, end node and
		// the intersections between them (this vector will be used for the Robot's navigation)s
		int Counter = 0;
		while(1)
		{
			//std::cout << "Node " << p->NodeName << " at x:" << p->x << " y:" << p->y << std::endl;
			NodesPath.push_back(p->NodeName);
			if (p->bSteppingStone == 0)
			{
				DestinationNodesPath.push_back(p->NodeName);
			}

			if (p->parent == nullptr)
			{
				IntersectionStartEndNodesPath.push_back(*p);
				break;
			}
			else if ((p->bIntersection == 1)||(Counter == 0))
			{
				IntersectionStartEndNodesPath.push_back(*p);
			}

			// Set next node to this node's parent
			p = p->parent;
			Counter = Counter + 1;
		}

	}
	return 0;
}

// Covert the sequence of nodes into a sequence of directions in the global reference frame
int PathFinder::DirectionsFromPath()
{
	int DeltaX;
	int DeltaY;

	for (int i = IntersectionStartEndNodesPath.size() - 1; i >= 0  ; i--)
	{
		if (i==IntersectionStartEndNodesPath.size() - 1)
		{
			//std::cout << "Start Node: " << (IntersectionStartEndNodesPath[i]).NodeName << std::endl;
			//std::cout << "Intersection Node Parent: " << (IntersectionStartEndNodesPath[i-1]).NodeName << std::endl;
		}
		else if (i==0)
		{
			//std::cout << "End Node: " << (IntersectionStartEndNodesPath[i]).NodeName << std::endl;
			break;
		}
		else
		{
			//std::cout << "Node: " << (IntersectionStartEndNodesPath[i]).NodeName << std::endl;
			//std::cout << "Intersection Node Parent: " << (IntersectionStartEndNodesPath[i-1]).NodeName << std::endl;
		}

		DeltaX = (IntersectionStartEndNodesPath[i-1]).x - (IntersectionStartEndNodesPath[i]).x;
		DeltaY = (IntersectionStartEndNodesPath[i-1]).y - (IntersectionStartEndNodesPath[i]).y;
		int up    = 1;
		int down  = 2;
		int left  = 3;
		int right = 4;

		// UNCOMMENT STD::COUT TO OUTPUT HUMAN READABLE DIRECTIONS
		if ((DeltaY == 0)&&(DeltaX != 0))
		{
			if (DeltaX > 0)
			{
				// std::cout << "right" << std::endl;
				Directions.push_back(right);
			}
			else if (DeltaX < 0)
			{
				// std::cout << "left" << std::endl;
				Directions.push_back(left);
			}
		}
		else if (DeltaY != 0)
		{
			if (DeltaY > 0)
			{
				// std::cout << "down" << std::endl;
				Directions.push_back(down);
			}
			else if (DeltaY < 0)
			{
				// std::cout << "up" << std::endl;
				Directions.push_back(up);
			}
		}
		else
		{
			std::cout << "error: unknown direction" << std::endl;
		}
    }
	return 0;
}

// Outputs the sequence of nodes along the shortest path between the start and end node
std::vector<std::string> PathFinder::GivePath(std::string StartNode, std::string EndNode)
{
	//std::vector<std::string> DestinationNodesPath;
	OnUserCreate(StartNode, EndNode);
	Solve_AStar();
	return(DestinationNodesPath);
}

// The public function: outputs the sequence of global directions along the shortest path between the input start and end node
std::vector<int> PathFinder::StartEndDirections(std::string StartNode, std::string EndNode)
{
	GivePath(StartNode, EndNode);
	DirectionsFromPath();
	return Directions;
}
