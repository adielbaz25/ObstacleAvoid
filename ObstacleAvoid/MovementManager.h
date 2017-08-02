/*
 * MovementManager.h
 *
 * Author: Adi Elbaz 206257313
 *         Yuval Ron 313584187
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
	Node * waypoint;
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
	void calculateTargetYaw(Node* waypoint);
	void stopMoving();
	void recalculateDeltaYaw();

public:
	MovementManager(Robot * robot, MapDrawer* mapDrawer);
	void NavigateToWaypoint(Node * waypoint);
	virtual ~MovementManager();
};

#endif /* MOVEMENTMANAGER_H_ */
