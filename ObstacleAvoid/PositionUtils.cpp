#ifndef UTILS_POSITIONUTILS_H_
#define UTILS_POSITIONUTILS_H_
#include "Structs.h"
#include <math.h>
#include "Node.h"
#include "Constants.h"

position RotateAroundOrigin(position coords, float theta)
{
    float thetaRad = theta / 180 * 3.141592;
    float cosineTheta = cos(thetaRad);
    float sineTheta = sin(thetaRad);
    position rotatedCoords;
    rotatedCoords.x=coords.x * cosineTheta - coords.y * sineTheta;
    rotatedCoords.y=coords.x * sineTheta + coords.y * cosineTheta;
    return rotatedCoords;
}

position toMapCoordinates(position worldCoords,int a , int b)
{

	    float scaleFactor = 1;
	    position center;
		center.x = -190;
		center.y = 197;

	    position rotated = RotateAroundOrigin(worldCoords, 330.0f); // 225 calculated by Idan

	    position scaled;
	    scaled.x = rotated.x * scaleFactor,
	    scaled.y = rotated.y * scaleFactor;

	    position translated;
	    translated.x= round(scaled.x + center.x),
	    translated.y= round(scaled.y + center.y);

	   return translated;
}

position toWorldCoordinates(position cell)
{
	float scaleFactor = 1;
	position center;
	center.x=470;
	center.y=470;

	position transnated;
	transnated.x= cell.x - center.x,
	transnated.y= cell.y - center.y;
	position scaled;

	scaled.x= transnated.x / scaleFactor;
	scaled.y= transnated.y / scaleFactor;

	position rotated = RotateAroundOrigin(scaled, 240.0f);

	return rotated;
}

Node ConvertToHamsterLocation(Node* waypoint)
{
	Node hamsterLocation(waypoint->getX()- ROBOT_START_X, waypoint->getY() - ROBOT_START_Y);

	return hamsterLocation;
}


#endif // UTILS_POSITIONUTILS_H_
