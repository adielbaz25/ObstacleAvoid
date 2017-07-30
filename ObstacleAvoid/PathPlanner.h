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
	double calculateDistance(MapCell* source, MapCell* target);
	void initializeHuristicValues(MapMatrix* map, MapCell* goalMapCell);
	bool isMapCellInList(set<MapCell>* list, int rowIndex, int colIndex);
	void handleNeighbors(MapMatrix* map, MapCell* currMapCell, MapCell* goalMapCell,
			set<MapCell*>* openList, set<MapCell*>* closedList);
	void findShortestPath(MapMatrix* map, MapCell* startMapCell, MapCell* goalMapCell);
	std::list<MapCell* > markWaypoints(MapCell * start, MapCell * currMapCell);

private:
	MapCell* getMinimalFMapCell(set<MapCell*>* openList);
	double getShipua(MapCell* a, MapCell* b);
};

#endif  PATHPLANNER_H_

