#include "Robot.h"
#include "LocalizationManager.h"

Robot::Robot(Hamster * hamster,
		LocalizationManager * localizationManager,
		int inflationRadius,
		double resolution)
{

	this->hamster = hamster;
	this->localizationManager = localizationManager;
	this->inflationRadius = inflationRadius;
	this->resolution = resolution;

	OccupancyGrid ogrid = hamster->getSLAMMap();

	this->mapHeight = ogrid.getHeight();
	this->mapWidth = ogrid.getWidth();

	this->prevX = 0;
	this->prevY = 0;
	this->prevYaw = 0;
	this->currX = 0;
	this->currY = 0;
	this->currYaw = 0;
}

void Robot::Initialize(positionState startLocation)
{
	hamsterStartX = startLocation.pos.x - (mapWidth / 2);
	hamsterStartY = startLocation.pos.y - (mapHeight / 2);

	currX = hamsterStartX;
	currY = hamsterStartY;
	currYaw = startLocation.yaw;

	Pose initialPose;
	initialPose.setX(hamsterStartX);
	initialPose.setY(hamsterStartY);
	initialPose.setHeading(startLocation.yaw);

	// The call to setInitialPose sets the parameter pose as the relative
	// origin of the robot's coordinate system.
	// The robot wouldn't start moving without this call, and after calling it - the values
	// returned from getPose would be relative to the initialPose we chose to set.
	sleep(3);
	hamster->setInitialPose(initialPose);

	cout << "Set Hamster initial pose at (" << hamsterStartX << ", " << hamsterStartY << ")" << endl;

	//localizationManager->InitParticles();

	UpdateLocation();
}

double Robot::GetDeltaX() const
{
	return currX - prevX;
}

double Robot::GetDeltaY() const
{
	return currY - prevY;
}

double Robot::GetDeltaYaw() const
{
	return currYaw - prevYaw;
}

positionState Robot::GetCurrHamsterLocation()
{
	/*Particle * topParticle = localizationManager->GetTopParticle();
	Location currLocation;
	currLocation = {
			.x = topParticle->x + 2*inflationRadius,
			.y = topParticle->y + 2*inflationRadius,
			.yaw = topParticle->yaw
	};*/

	Pose currPose = hamster->getPose();

	// Substract the initial pose coordinates from the current pose to get
	// coordinates relative to (0,0) instead of to the initial pose we set
	double currX = (currPose.getX() - hamsterStartX) / resolution;
	double currY = ((currPose.getY() * -1) - hamsterStartY) / resolution;

	double currYaw = currPose.getHeading();

	if (currYaw < 0)
	{
		currYaw += 360;
	}
	else if (currYaw > 360)
	{
		currYaw -= 360;
	}

	position currentPos = { .x = currX, .y = currY };
	positionState currPosState = { .pos = currentPos, .yaw = 360 - currYaw };

	return currPosState;
}

void Robot::UpdateLocation()
{
	prevX = currX;
	prevY = currY;
	prevYaw = currYaw;

	positionState currentLocation = GetCurrHamsterLocation();

	// Update the current and previous locations by the position of the robot
	currX = currentLocation.pos.x;
	currY = currentLocation.pos.y;
	currYaw = currentLocation.yaw;
}

Robot::~Robot() {
	// TODO Auto-generated destructor stub
}
