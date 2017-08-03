#include "MovementManager.h"
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <sstream>

#define MAP_ANGLE 0

#define MAX_MOVE_SPEED 0.4
#define MIN_TURN_SPEED 0.1
#define MAX_TURN_SPEED 0.2

#define YAW_TOLERANCE 1.05
#define NAVIGATION_TIMEOUT_IN_SECONDS 15

#define TURN_ANGLE 45.0

#define MAX_WHEELS_ANGLE 45
#define MIN_WHEELS_ANGLE 2

//#define leftTurnAngle()		TURN_ANGLE
//#define rightTurnAngle()	-TURN_ANGLE

MovementManager::MovementManager(HamsterAPI::Hamster * hamster, Robot * robot, MapDrawer* mapDrawer, LocalizationManager* localizationManager)
{
	this->hamster = hamster;
	this->robot = robot;
	this->mapDrawer = mapDrawer;
	this->localizationManager = localizationManager;
}

void MovementManager::StopMoving()
{
	hamster->sendSpeed(0.0, 0.0);
}

clock_t MovementManager::isRobotStuck() {
	return ((clock() - navigationStartTime) / CLOCKS_PER_SEC) >= NAVIGATION_TIMEOUT_IN_SECONDS;
}

bool MovementManager::isDeltaAngleOnEndOfCiricle() {
	return (destYaw > (360 - YAW_TOLERANCE) && currYaw < YAW_TOLERANCE) ||
		   (currYaw > (360 - YAW_TOLERANCE) && destYaw < YAW_TOLERANCE);
}

bool MovementManager::isRequiredAngleAdjustment()
{
	return !isDeltaAngleOnEndOfCiricle() || currDeltaYaw > YAW_TOLERANCE;
}

void MovementManager::calculateDestYaw(Node* waypoint)
{
	double destYawInRad = atan2((waypoint->getY() - currLocation.pos.y),
	                            (waypoint->getX() - currLocation.pos.x));
	double destYawInDegrees = AngleUtils::convertRadiansToDegrees(destYawInRad) + MAP_ANGLE;
	destYaw = GetAdjustedYaw(destYawInDegrees);
}

// The parameter waypoint should be according to Hamster's coordinate system
// (in which (0,0) is at the center of the map, NOT at the top left corner)
void MovementManager::NavigateToWaypoint(Node* waypoint, LidarScan& scan)
{
	this->waypoint = waypoint;
	currLocation = robot->GetCurrHamsterLocation();

	calculateDestYaw(waypoint);
	RecalculateDistanceFromWaypoint();
	//RecalculateTurningDirection();

	locationChanged = true;
	bool isWaypointReached = false;

	navigationStartTime = clock();

	while (!isWaypointReached)
	{
		currLocation = robot->GetCurrHamsterLocation();
		currYaw = currLocation.yaw;
		currDeltaYaw = fabs(destYaw - currYaw);
		calculateDestYaw(waypoint);

		if (isRequiredAngleAdjustment())
		{
			cout << "Turning, destYaw: " << destYaw << " currYaw: " << currYaw << " deltaYaw: " << currDeltaYaw << " w: (" << waypoint->getX() << ", " << waypoint->getY() <<
					") r: (" << currLocation.pos.x << ", " << currLocation.pos.y << ")" << endl;
			RecalculateTurningDirection();
			TurnToWaypoint();
		}
		else
		{
			// Keep moving in the chosen direction while the robot is not too close to the waypoint
			cout << "Forward, waypoint: (" << waypoint->getX() << ", " << waypoint->getY() <<
					") robot: (" << currLocation.pos.x << ", " << currLocation.pos.y << ")" << endl;

			MoveToWaypoint();
		}

		localizationManager->BestParticle(&scan, -1 * (hamster->getPose().getY() / 0.05) + 470, hamster->getPose().getX() / 0.05 + 470);

		mapDrawer->RevertToSavedMap();
		mapDrawer->DrawPatricles(&(localizationManager->particles));

		RecalculateDistanceFromWaypoint();
		isWaypointReached = distanceFromWaypoint <= DISTANCE_FROM_WAYPOINT_TOLERANCE;
		mapDrawer->Show();
		sleep(1.5);
	}

	PrintAfterWaypointIsReached();
	StopMoving();

	return;
}

void MovementManager::TurnToWaypoint()
{
	CalculateTurnSpeedByDeltaYaw();
	hamster->sendSpeed(turnSpeed, wheelsAngle);

	prevLocation = currLocation;
	currLocation = robot->GetCurrHamsterLocation();
	currYaw = currLocation.yaw;

	locationChanged =
		prevLocation.pos.x != currLocation.pos.x &&
		prevLocation.pos.y != currLocation.pos.y &&
		prevLocation.yaw != currLocation.yaw;

	if (locationChanged)
	{
		chosenDirectionName = wheelsAngle > 0 ? "Left" : "Right";
		PrintAfterTurning();
	}
}

void MovementManager::MoveToWaypoint()
{
	CalculateMoveSpeedByDistanceFromWaypoint();
	hamster->sendSpeed(moveSpeed, 0.0);

	currLocation = robot->GetCurrHamsterLocation();
	prevDistanceFromWaypoint = distanceFromWaypoint;
	RecalculateDistanceFromWaypoint();

	locationChanged = prevDistanceFromWaypoint != distanceFromWaypoint;

	if (locationChanged)
	{
		chosenDirectionName = "Forward";
		PrintAfterMoving();
	}
}

float MovementManager::calculateWheelsAngle()
{
	if(currDeltaYaw > MAX_WHEELS_ANGLE)
	{
		return MAX_WHEELS_ANGLE;
	}

	if(currDeltaYaw < MIN_WHEELS_ANGLE)
	{
		return MIN_WHEELS_ANGLE;
	}

	return currDeltaYaw;
}

void MovementManager::RecalculateTurningDirection()
{
	if (currYaw > destYaw)
	{
		if (360 - currYaw + destYaw < currYaw - destYaw)
		{
			wheelsAngle = calculateWheelsAngle() * -1;
		}
		else
		{
			wheelsAngle = calculateWheelsAngle();
		}
	}
	else
	{
		if (360 - destYaw + currYaw < destYaw - currYaw)
		{
			wheelsAngle = calculateWheelsAngle();
		}
		else
		{
			wheelsAngle = calculateWheelsAngle() * -1;
		}
	}
}

double MovementManager::GetAdjustedYaw(double yawToAdjust) const
{
	if (yawToAdjust < 0)
	{
		return yawToAdjust + 360;
	}

	if (yawToAdjust > 360)
	{
		return yawToAdjust - 360;
	}

	return yawToAdjust;
}

void MovementManager::RecalculateDistanceFromWaypoint()
{
	currLocation = robot->GetCurrHamsterLocation();

	distanceFromWaypoint =
		sqrt(pow(currLocation.pos.x - waypoint->getX(), 2) +
			 pow(currLocation.pos.y - waypoint->getY(), 2));
}

// Calculate the turn speed according to the current delta yaw in a circular way (0 = 360),
// so that the robot would turn slower as it approaches the destination yaw in order not
// to miss it
void MovementManager::CalculateTurnSpeedByDeltaYaw()
{
	int numOfSpeedClasses = 5;
	double diffConst = 2 * ((double)(MAX_TURN_SPEED - MIN_TURN_SPEED) / (numOfSpeedClasses - 1));

	double classSize = (double)360.0 / numOfSpeedClasses;

	double divisionResult = (double)currDeltaYaw / classSize;

	// Varies from (0) to (numOfSpeedClasses - 1)
	int speedClassIndex = floor(divisionResult);

	if (speedClassIndex > ((int)(numOfSpeedClasses / 2)))
	{
		turnSpeed = MIN_TURN_SPEED + (numOfSpeedClasses - speedClassIndex) * diffConst;
	}
	else
	{
		turnSpeed = MIN_TURN_SPEED + speedClassIndex * diffConst;
	}
}

// Calculate the move speed according to the current distance from the waypoint,
// so that the robot would move slower as it approached the waypoint in order not
// to miss it
void MovementManager::CalculateMoveSpeedByDistanceFromWaypoint()
{
	if (distanceFromWaypoint > 5 * DISTANCE_FROM_WAYPOINT_TOLERANCE)
	{
		moveSpeed = MAX_MOVE_SPEED;
	}

	moveSpeed = (double)distanceFromWaypoint / 50;
}

void MovementManager::MoveBackwards()
{
	cout << "Moving Backwards" << endl;

	// Move backwards trying to avoid the obstacle that the robot got stuck in
	for (int i = 0; i < 1000000; i++)
	{
		hamster->sendSpeed(-0.15, 0.0);
	}

	navigationStartTime = clock();
}

void MovementManager::PrintBeforeTurning()
{
	cout << "Preparing to turn..." << endl
		<< "current location: " << "x = " << currLocation.pos.x << ", y = " << currLocation.pos.y
		<< ", yaw = " << currYaw << endl
		<< "current waypoint: " << "x = " << waypoint->getX() << ", y = " << waypoint->getY() << endl
		<< "destYaw: " << destYaw << endl;
}

void MovementManager::PrintAfterTurning()
{
	cout << "Turned " << chosenDirectionName << " to: " <<
		"x = " << currLocation.pos.x <<
		", y = " << currLocation.pos.y <<
		", yaw = " << currYaw <<
		", deltaYaw = " << currDeltaYaw <<
		" (turnSpeed: " << turnSpeed << ")" << endl;
}

void MovementManager::PrintAfterMoving()
{
	cout << "Moved " << chosenDirectionName << " to: " <<
		"x = " << currLocation.pos.x <<
		", y = " << currLocation.pos.y <<
		", yaw = " << currYaw <<
		", distanceFromWaypoint =  " << distanceFromWaypoint <<
		" (moveSpeed: " << moveSpeed << ")" << endl;
}

void MovementManager::PrintAfterWaypointIsReached()
{
	cout << endl <<
		"Reached waypoint (" << waypoint->getX() << ", " << waypoint->getY() << ")" << endl <<
		"current location: " <<
		"x = " << currLocation.pos.x <<
		", y = " << currLocation.pos.y <<
		", yaw = " << currLocation.yaw << endl << endl;
}

MovementManager::~MovementManager() {
	// TODO Auto-generated destructor stub
}
