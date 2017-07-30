#include "Robot.h"

Robot::Robot(Hamster * hamster,
		LocalizationManager * localizationManager,
		double mapHeight,
		double mapWidth,
		double resolution)
{
	this->hamster = hamster;
	this->localizationManager = localizationManager;
	this->resolution = resolution;

	this->mapHeight = mapHeight;
	this->mapWidth = mapWidth;
}

positionState Robot::GetRealHamsterLocation()
{
	Pose currPose = hamster->getPose();
	double currX = (currPose.getX()) / resolution;
	double currY = ((currPose.getY() * -1)) / resolution;
	double currYaw = getYawInOneCiricle(currPose.getHeading());

	position currentPos = { .x = currX, .y = currY };
	positionState currPosState = { .pos = currentPos, .yaw = 360 - currYaw };

	return currPosState;
}

void Robot::updateHamsterRealLocation()
{
	realLocation = GetRealHamsterLocation();
}

void Robot::setHamsterSpeed(double speed, double wheelsAngle)
{
	this->hamster->sendSpeed(speed, wheelsAngle);
}

Robot::~Robot() {
}

