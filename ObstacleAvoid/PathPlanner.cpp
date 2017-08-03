/*
 * PathPlanner.cpp
 *
 * Author: Adi Elbaz 206257313
 *         Yuval Ron 313584187
 */

#include "PathPlanner.h"

void PathPlanner::findShortestPath(MapMatrix* map, Node* startNode, Node* goalNode)
{
	set<Node*> openList;
	set<Node*> closedList;
	Node* currNode;

	// Initalize the h* value of the nodes
	initializeHuristicValues(map, goalNode);

	// Calculate the f, g, h of the first node neighbors
	handleNeighbors(map, startNode, goalNode, &openList, &closedList);

	// Insert the source node to the closeList
	closedList.insert(startNode);
	startNode->setIsInClosedList(true);

	while(!openList.empty())
	{
		// Calculate the next node with the minimum F*
		currNode = getMinimalFNode(&openList);

		// Delete the node from the open list and add it to the close list
		openList.erase(currNode);
		currNode->setIsInOpenList(false);
		closedList.insert(currNode);
		currNode->setIsInClosedList(true);

		// Calculate the f, g, h of the current node neighbors
		handleNeighbors(map, currNode, goalNode, &openList, &closedList);
	}
}

void PathPlanner::initializeHuristicValues(MapMatrix* map, Node* goalNode)
{
	for (int rowIndex = 0; rowIndex < map->getHeight(); rowIndex++)
	{
		for (int colIndex = 0; colIndex < map->getWidth(); colIndex++)
		{
			Node* currNode = map->getNodeAtIndex(colIndex, rowIndex);

			if (!currNode->getIsObstacle())
			{
				currNode->setH(calculateDistance(currNode, goalNode));
			}
		}
	}
}

double PathPlanner::calculateDistance(Node* source, Node* target)
{
	return sqrt(pow(source->getX() - target->getX(), 2) + pow(source->getY() - target->getY(), 2));
}

void PathPlanner::handleNeighbors(MapMatrix* map, Node* currNode, Node* goalNode,
		set<Node*>* openList, set<Node*>* closedList)
{
	for (int rowIndex = currNode->getY() - 1; rowIndex <= currNode->getY() + 1; rowIndex++)
	{
		for (int colIndex = currNode->getX() - 1; colIndex <= currNode->getX() + 1; colIndex++)
		{
			// Checks if we're out of bounds and if the current neighbor is not an obstacle
			if (colIndex >= 0 && rowIndex >= 0 &&
				colIndex < map->getWidth() && rowIndex < map->getHeight() &&
				!map->getNodeAtIndex(colIndex, rowIndex)->getIsObstacle())
			{
				// Makes sure the current Node is not scanned
				if (colIndex != currNode->getX() || rowIndex != currNode->getY())
				{
					Node* currNeighbor = map->getNodeAtIndex(colIndex, rowIndex);

					// Checks if the current neighbor is not in the closed list
					if (!currNeighbor->getIsInClosedList())
					{
						// Calculate the current node distance to its neighbor
						double tempGCost =
								calculateDistance(currNode, currNeighbor) + currNode->getG();

						// Checks if the current neighbor is not in the open list
						if (!currNeighbor->getIsInOpenList())
						{
							// Initialize the values of the node for the first time
							currNeighbor->setG(tempGCost);
							currNeighbor->setF(currNeighbor->getH() + tempGCost);
							currNeighbor->setParent(currNode);

							// Checking if we have reached the goal
							if (goalNode->getX() == colIndex && goalNode->getY() == rowIndex)
							{
								openList->clear();
								return;
							}

							// Adding the Node to the open list for the first time
							openList->insert(currNeighbor);
							currNeighbor->setIsInOpenList(true);
						}

						// The Node was already in the open list, check if we found a shorter path
						else
						{
							if (tempGCost < currNeighbor->getG())
							{
								currNeighbor->setG(tempGCost);
								currNeighbor->setF(currNeighbor->getH() + tempGCost);
								currNeighbor->setParent(currNode);
							}
						}
					}
				}
			}
		}
	}
}

Node* PathPlanner::getMinimalFNode(set<Node*>* openList)
{
	Node* minNode = *(openList->begin());
	Node* currNode;

	for (set<Node*>::iterator iter = openList->begin(); iter != openList->end(); iter++)
	{
		currNode = *iter;

		if (currNode->getF() < minNode->getF())
		{
			minNode = currNode;
		}
	}

	return minNode;
}

std::list<Node* > PathPlanner::markWaypoints(Node * start, Node * DestNode)
{
	std::list<Node* > waypoints;
	Node* firstNode, *secondNode, *thirdNode;
	firstNode = DestNode;
	secondNode = DestNode->getParent();
	int skipCounter = 0;

	// Check if the destination node is not the start node
	if(secondNode == NULL)
	{
		return waypoints;
	}

	// Loop on all the nodes in the path, from the destination to the first node
	while (firstNode->getX() != start->getX() || firstNode->getY() != start->getY())
	{
		// Initialize the parent node of the current parent
		thirdNode = secondNode->getParent();

		// Check if we reached to the destination node
		if(thirdNode == NULL)
		{
			waypoints.push_back(secondNode);
			return waypoints;
		}

		// Check if there is an angle between the points or if its between the size to add waypoint
		if((getIncline(firstNode,secondNode) != getIncline(secondNode,thirdNode)) ||
				skipCounter >= WAYPOINT_TOLERENCE)
		{
			secondNode->setIsWaypoint(true);
			waypoints.push_back(secondNode);
			skipCounter = 0;
		}
		else
		{
			skipCounter++;
		}

		firstNode = secondNode;
		secondNode = thirdNode;
	}

	return waypoints;
}

double PathPlanner::getIncline(Node *a, Node * b)
{
	if (b->getX() - a->getX() == 0)
		return 0;

	return (b->getY() - a->getY()) / (b->getX() - a->getX());
}
