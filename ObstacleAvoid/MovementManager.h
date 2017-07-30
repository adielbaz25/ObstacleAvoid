/*
 * MovementManager.h
 *
 *  Created on: Jul 2, 2017
 *      Author: user
 */

#ifndef MOVEMENTMANAGER_H_
#define MOVEMENTMANAGER_H_

#include "Robot.h"
#include "HamsterAPIClientCPP/Hamster.h"
#include "AngleUtils.h"
#include <vector>
#include <math.h>
#include "Constants.h"
using namespace std;
using namespace HamsterAPI;

#define DISTANCE_FROM_WAYPOINT_TOLERANCE 5

class MovementManager
{
private:
	Robot * robot;
	MapDrawer* mapDrawer;
	MapCell * waypoint;
	double distanceFromWaypoint;
	double targetYaw, deltaYaw;

	void turnToWaypoint();
	void moveToWaypoint();
	double GetAdjustedYaw(double yawToAdjust) const;
	double calculateTurningDirection();
	void recalculateDistanceFromWaypoint();
	double calculateTurnSpeed();
	double calculateForwardSpeed();
	bool isRequiredAngleAdjustment();
	bool isDeltaAngleOnEndOfCiricle();
	float calculateWheelsAngle();
	void calculateTargetYaw(MapCell* waypoint);
	void stopMoving();
	void recalculateDeltaYaw();

public:
	MovementManager(Robot * robot, MapDrawer* mapDrawer);
	void NavigateToWaypoint(MapCell * waypoint);
	virtual ~MovementManager();
};

#endif /* MOVEMENTMANAGER_H_ */
