#ifndef PATHPLANNER_H_
#define PATHPLANNER_H_

#include <set>
#include <math.h>
#include "Structs.h"
#include "NodeMap.h"
#include "Constants.h"

using namespace std;

class PathPlanner
{
public:
	double calculateDistance(Node* source, Node* target);
	void initializeHuristicValues(NodeMap* map, Node* goalNode);
	bool isNodeInList(set<Node>* list, int rowIndex, int colIndex);
	void handleNeighbors(NodeMap* map, Node* currNode, Node* goalNode,
			set<Node*>* openList, set<Node*>* closedList);
	void findShortestPath(NodeMap* map, Node* startNode, Node* goalNode);
	std::list<Node* > markWaypoints(Node * start, Node * currNode);

private:
	Node* getMinimalFNode(set<Node*>* openList);
	double getShipua(Node* a, Node* b);
};

#endif  PATHPLANNER_H_

