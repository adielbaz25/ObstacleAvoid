/*
 * PathPlanner.h
 *
 * Author: Adi Elbaz 206257313
 *         Yuval Ron 313584187
 */

#ifndef PATHPLANNER_H_
#define PATHPLANNER_H_

#include <set>
#include <math.h>
#include "Structs.h"
#include "MapMatrix.h"
#include "Constants.h"

using namespace std;

class PathPlanner
{
public:
	double calculateDistance(Node* source, Node* target);
	void initializeHuristicValues(MapMatrix* map, Node* goalNode);
	bool isNodeInList(set<Node>* list, int rowIndex, int colIndex);
	void handleNeighbors(MapMatrix* map, Node* currNode, Node* goalNode,
			set<Node*>* openList, set<Node*>* closedList);
	void findShortestPath(MapMatrix* map, Node* startNode, Node* goalNode);
	std::list<Node* > markWaypoints(Node * start, Node * DestNode);

private:
	Node* getMinimalFNode(set<Node*>* openList);
	double getIncline(Node* a, Node* b);
};

#endif  PATHPLANNER_H_

