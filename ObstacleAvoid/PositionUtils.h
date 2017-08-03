/*
 * PositionUtils.h
 *
 *  Created on: Jul 8, 2017
 *      Author: user
 */

#ifndef UTILS_POSITIONUTILS_H_
#define UTILS_POSITIONUTILS_H_
#include "PositionUtils.h"
#include "Structs.h"
#include <math.h>
#include "Node.h"

position RotateAroundOrigin(position coords, float theta);
position toMapCoordinates(position worldCoords,int a, int b);
position toWorldCoordinates(position cell);
Node ConvertToHamsterLocation(Node* waypoint);

#endif /* UTILS_POSITIONUTILS_H_ */
