#ifndef UTILS_POSITIONUTILS_H_
#define UTILS_POSITIONUTILS_H_
#include "Structs.h"
#include <math.h>
#include "MapCell.h"
#include "Constants.h"

MapCell ConvertToHamsterLocation(MapCell* waypoint)
{
	MapCell hamsterLocation(waypoint->getX()- ROBOT_START_X, waypoint->getY() - ROBOT_START_Y);

	return hamsterLocation;
}


#endif  UTILS_POSITIONUTILS_H_
