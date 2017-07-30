#include "PathPlanner.h"

void PathPlanner::findShortestPath(MapMatrix* map, MapCell* startMapCell, MapCell* goalMapCell)
{
	set<MapCell*> openList;
	set<MapCell*> closedList;
	MapCell* currMapCell;

	initializeHuristicValues(map, goalMapCell);
	closedList.insert(startMapCell);
	startMapCell->setIsInClosedList(true);
	handleNeighbors(map, startMapCell, goalMapCell, &openList, &closedList);

	while(!openList.empty())
	{
		currMapCell = getMinimalFMapCell(&openList);

		openList.erase(currMapCell);
		currMapCell->setIsInOpenList(false);
		closedList.insert(currMapCell);
		currMapCell->setIsInClosedList(true);

		handleNeighbors(map, currMapCell, goalMapCell, &openList, &closedList);

	}
}

void PathPlanner::initializeHuristicValues(MapMatrix* map, MapCell* goalMapCell)
{
	for (int rowIndex = 0; rowIndex < map->getHeight(); rowIndex++)
	{
		for (int colIndex = 0; colIndex < map->getWidth(); colIndex++)
		{
			MapCell* currMapCell = map->getMapCellAtIndex(colIndex, rowIndex);

			if (!currMapCell->getIsObstacle())
			{
				currMapCell->setH(calculateDistance(currMapCell, goalMapCell));
			}
		}
	}
}

double PathPlanner::calculateDistance(MapCell* source, MapCell* target)
{
	return sqrt(pow(source->getX() - target->getX(), 2) + pow(source->getY() - target->getY(), 2));
}

void PathPlanner::handleNeighbors(MapMatrix* map, MapCell* currMapCell, MapCell* goalMapCell,
		set<MapCell*>* openList, set<MapCell*>* closedList)
{
	for (int rowIndex = currMapCell->getY() - 1; rowIndex <= currMapCell->getY() + 1; rowIndex++)
	{
		for (int colIndex = currMapCell->getX() - 1; colIndex <= currMapCell->getX() + 1; colIndex++)
		{
			//cout << "X: " << colIndex << " Y: " << rowIndex << endl;

			// Checks if we're out of bounds and if the current neighbor is not an obstacle
			if (colIndex >= 0 && rowIndex >= 0 &&
				colIndex < map->getWidth() && rowIndex < map->getHeight() &&
				!map->getMapCellAtIndex(colIndex, rowIndex)->getIsObstacle())
			{
				// Makes sure the current MapCell is not scanned
				if (colIndex != currMapCell->getX() || rowIndex != currMapCell->getY())
				{
					// Checks if the current neighbor is in the closed list

					//if (!isMapCellInList(closedList, colIndex, rowIndex))
					//{
					MapCell* currNeighbor = map->getMapCellAtIndex(colIndex, rowIndex);

					if (!currNeighbor->getIsInClosedList())
					{
						double tempGCost =
							calculateDistance(currMapCell, currNeighbor) + currMapCell->getG();

						// Checks if the current neighbor is already in the open list
						if (!currNeighbor->getIsInOpenList())
						{
							currNeighbor->setG(tempGCost);
							currNeighbor->setF(currNeighbor->getH() + tempGCost);
							currNeighbor->setParent(currMapCell);

							// Checking if we have reached the goal
							if (goalMapCell->getX() == colIndex && goalMapCell->getY() == rowIndex)
							{
								openList->clear();
								return;
							}

							// Adding the MapCell to the open list for the first time
							openList->insert(currNeighbor);
							currNeighbor->setIsInOpenList(true);
						}
						// The MapCell was already in the open list, check if we found a shorter path
						else
						{
							if (tempGCost < currNeighbor->getG())
							{
								currNeighbor->setG(tempGCost);
								currNeighbor->setF(currNeighbor->getH() + tempGCost);
								currNeighbor->setParent(currMapCell);
							}
						}
					}
				}
			}
		}
	}
}

MapCell* PathPlanner::getMinimalFMapCell(set<MapCell*>* openList)
{
	MapCell* minMapCell = *(openList->begin());
	MapCell* currMapCell;

	for (set<MapCell*>::iterator iter = openList->begin(); iter != openList->end(); iter++)
	{
		currMapCell = *iter;

		if (currMapCell->getF() < minMapCell->getF())
		{
			minMapCell = currMapCell;
		}
	}

	return minMapCell;
}

std::list<MapCell* > PathPlanner::markWaypoints(MapCell * start, MapCell * currMapCell)
{
	std::list<MapCell* > waypoints;
	MapCell* firstMapCell, *secondMapCell, *thirdMapCell;
	firstMapCell = currMapCell;
	secondMapCell = currMapCell->getParent();
	int skipCounter = 0;

	if(secondMapCell == NULL)
	{
		return waypoints;
	}

	while (firstMapCell->getX() != start->getX() || firstMapCell->getY() != start->getY())
	{

		thirdMapCell = secondMapCell->getParent();

		if(thirdMapCell == NULL)
		{
			waypoints.push_back(secondMapCell);
			return waypoints;
		}

		if((getShipua(firstMapCell,secondMapCell) != getShipua(secondMapCell,thirdMapCell)) ||
				skipCounter >= WAYPOINT_TOLERENCE)
		{
			secondMapCell->setIsWaypoint(true);
			waypoints.push_back(secondMapCell);
			skipCounter = 0;
		}
		else
		{
			skipCounter++;
		}


		firstMapCell = secondMapCell;
		secondMapCell = thirdMapCell;
	}

	return waypoints;
}

double PathPlanner::getShipua(MapCell *a, MapCell * b)
{
	if (b->getX() - a->getX() == 0)
		return 0;

	return (b->getY() - a->getY()) / (b->getX() - a->getX());
}



