/*
 * MovementManager.cpp
 *
 * Author: Adi Elbaz 206257313
 *         Yuval Ron 313584187
 */

#include "MovementManager.h"
#include <stdio.h>
#include <stdlib.h>
#include <sstream>

MovementManager::MovementManager(Robot * robot, MapDrawer* mapDrawer)
{
	this->robot = robot;
	this->mapDrawer = mapDrawer;
}

void MovementManager::NavigateToWaypoint(Node* waypoint)
{
	this->waypoint = waypoint;
	robot->realLocation = robot->GetRealHamsterLocation();

	recalculateDistanceFromWaypoint();

	while (distanceFromWaypoint > DISTANCE_FROM_WAYPOINT_TOLERANCE)
	{
		robot->prevBeliefedLocation = robot->currBeliefedLocation;
		robot->realLocation = robot->currBeliefedLocation = robot->GetRealHamsterLocation();
		recalculateDeltaYaw();

		calculateTargetYaw(waypoint);

		if (isRequiredAngleAdjustment())
		{
			cout << "Turning, targetYaw: " << targetYaw << " currYaw: " << robot->currBeliefedLocation.yaw << " deltaYaw: " << deltaYaw << " w: (" << waypoint->getX() << ", " << waypoint->getY() <<
					") r: (" << robot->currBeliefedLocation.pos.x << ", " << robot->currBeliefedLocation.pos.y << ")" << endl;
			turnToWaypoint();
		}
		else
		{
			cout << "Forward, waypoint: (" << waypoint->getX() << ", " << waypoint->getY() <<
					") robot: (" << robot->currBeliefedLocation.pos.x << ", " << robot->currBeliefedLocation.pos.y << ")" << endl;

			moveToWaypoint();
		}

		recalculateDistanceFromWaypoint();
		mapDrawer->Show(robot->GetRealHamsterLocation());
		sleep(1.5);
	}

	cout << "The waypoint: (" << waypoint->getX() << ", " << waypoint->getY() << ") has been reached" << endl;
	stopMoving();

	return;
}


void MovementManager::recalculateDeltaYaw()
{
	// Is in circle
	if((targetYaw > 360 - DELTA_ANGLE_TOL && robot->currBeliefedLocation.yaw < DELTA_ANGLE_TOL) ||
	   (robot->currBeliefedLocation.yaw > 360 - DELTA_ANGLE_TOL && targetYaw < DELTA_ANGLE_TOL))
	{
		deltaYaw = fabs((targetYaw + robot->currBeliefedLocation.yaw) - 360);
	}
	else
	{
		deltaYaw = fabs(targetYaw - robot->currBeliefedLocation.yaw);
	}
}

bool MovementManager::isRequiredAngleAdjustment()
{
	return !isDeltaAngleOnEndOfCiricle() && deltaYaw > YAW_TOLERANCE;
}

// Forward movement more stable
bool MovementManager::isDeltaAngleOnEndOfCiricle() {
	return (targetYaw > (360 - YAW_TOLERANCE) && robot->currBeliefedLocation.yaw < YAW_TOLERANCE) ||
		   (robot->currBeliefedLocation.yaw > (360 - YAW_TOLERANCE) && targetYaw < YAW_TOLERANCE);
}

void MovementManager::calculateTargetYaw(Node* waypoint)
{
	targetYaw = getYawInOneCiricle(convertRadiansToDegrees(atan2((waypoint->getY() - robot->currBeliefedLocation.pos.y),
																(waypoint->getX() - robot->currBeliefedLocation.pos.x))));
}

void MovementManager::turnToWaypoint()
{
	robot->setHamsterSpeed(calculateTurnSpeed(), calculateTurningDirection());
}

void MovementManager::moveToWaypoint()
{
	robot->setHamsterSpeed(calculateForwardSpeed(), 0.0);
}

void MovementManager::stopMoving()
{
	robot->setHamsterSpeed(0.0, 0.0);
}

double MovementManager::calculateTurningDirection()
{
	if (robot->currBeliefedLocation.yaw > targetYaw)
	{
		if (360 - robot->currBeliefedLocation.yaw + targetYaw <
					robot->currBeliefedLocation.yaw - targetYaw)
		{
			return calculateWheelsAngle() * -1;
		}
		else
		{
			return calculateWheelsAngle();
		}
	}
	else
	{
		if (360 - targetYaw + robot->currBeliefedLocation.yaw <
					targetYaw - robot->currBeliefedLocation.yaw)
		{
			return calculateWheelsAngle();
		}
		else
		{
			return calculateWheelsAngle() * -1;
		}
	}
}

float MovementManager::calculateWheelsAngle()
{
	if(deltaYaw > MAX_WHEELS_ANGLE)
	{
		return MAX_WHEELS_ANGLE;
	}

	if(deltaYaw < MIN_WHEELS_ANGLE)
	{
		return MIN_WHEELS_ANGLE;
	}

	return deltaYaw;
}

void MovementManager::recalculateDistanceFromWaypoint()
{
	distanceFromWaypoint =
		sqrt(pow(robot->currBeliefedLocation.pos.x - waypoint->getX(), 2) +
			 pow(robot->currBeliefedLocation.pos.y - waypoint->getY(), 2));
}

double MovementManager::calculateTurnSpeed()
{
	double yawRatio = deltaYaw / targetYaw;
	if(yawRatio > 1)
	{
		yawRatio = 1 / yawRatio;
	}

	return MIN_TURN_SPEED + ((MAX_TURN_SPEED - MIN_TURN_SPEED) *  yawRatio);
}

double MovementManager::calculateForwardSpeed()
{
	if (distanceFromWaypoint > MIN_DISTANCE_FOR_FULL_SPEED)
	{
		return MAX_MOVE_SPEED;
	}

	return (double)distanceFromWaypoint / FORWARD_SPEED_FACTOR;
}

MovementManager::~MovementManager() {
}
