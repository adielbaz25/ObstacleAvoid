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
#include "LocalizationManager.h"
using namespace std;
using namespace HamsterAPI;

#define DISTANCE_FROM_WAYPOINT_TOLERANCE 5

class MovementManager
{
public:
    MovementManager(HamsterAPI::Hamster * hamster, Robot * robot, MapDrawer* mapDrawer, LocalizationManager* localizationManager);
    void NavigateToWaypoint(Node * waypoint, LidarScan& scan);
    void StopMoving() ;
    virtual ~MovementManager();

private:
	HamsterAPI::Hamster * hamster;
	Robot * robot;
	MapDrawer* mapDrawer;
	positionState currLocation;
	positionState prevLocation;
	Node * waypoint;
	double distanceFromWaypoint, prevDistanceFromWaypoint;
	double currYaw, destYaw, currDeltaYaw;
	double turnSpeed, moveSpeed;
	string chosenDirectionName;
	clock_t navigationStartTime;
	float wheelsAngle;
	bool locationChanged;
	LocalizationManager* localizationManager;

	void TurnToWaypoint();
	void MoveToWaypoint();

	double GetAdjustedYaw(double yawToAdjust) const;
	void RecalculateTurningDirection();
	void RecalculateDistanceFromWaypoint();
	void CalculateTurnSpeedByDeltaYaw();
	void CalculateMoveSpeedByDistanceFromWaypoint();
	void MoveBackwards();
	void PrintBeforeTurning();
	void PrintAfterTurning();
	void PrintAfterMoving();
	void PrintAfterWaypointIsReached();
	bool isRequiredAngleAdjustment();
	clock_t isRobotStuck();
	bool isDeltaAngleOnEndOfCiricle();
	float calculateWheelsAngle();
	void calculateDestYaw(Node* waypoint);
};

#endif /* MOVEMENTMANAGER_H_ */
