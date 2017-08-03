#include "LocalizationParticle.h"
#include "LocalizationManager.h"

#include <HamsterAPIClientCPP/Hamster.h>
#include <iostream>
#include "Constants.h"


using namespace std;

// Create a new child-particle.
LocalizationParticle* LocalizationParticle::CreateChild(float expansionRadius, float yawRange)
{
    float newX = this->x + Random(-expansionRadius, expansionRadius);
    float newY = this->y + Random(-expansionRadius, expansionRadius);
    float newYaw = this->yaw + Random(-yawRange, yawRange);

	LocalizationParticle* newParticle = new LocalizationParticle(newX, newY, newYaw, -1);
	newParticle->hamster = this->hamster;
	return newParticle;
}

float LocalizationParticle::Random(float min, float max)
{
    float num = (float)rand() / RAND_MAX;
    return min + num * (max - min);
}

// Create child-particle.
LocalizationParticle* LocalizationParticle::CreateChild()
{
    return CreateChild(EXPANSION_RADIUS, YAW_RANGE);
}

// Update the particle
void LocalizationParticle::Update(float xDelta, float yDelta, float yawDelta, MapMatrix* map, LidarScan* lidarHandler, Robot* amit)
{
	this->x += xDelta;
    this->y += yDelta;
    this->yaw += yawDelta;

    // Calculating the belief of the particle, by using the probability by movement and laser scan.
    float predictionBelif = ProbabilityByMovement(xDelta, yDelta, yawDelta) * this->belief;
    float probabilityByScan = ProbabilityByLidarScan(*map, *amit);//manager->computeBelief(this, *lidarHandler);
    this->belief = predictionBelif * predictionBelif * BELIEF_NORMALIZATION_VALUE;

    // In case the belief is more than 1, put 1 instead.
    if (this->belief > 1)
    {
    	this->belief = 1;
    }
}

// calculate the distance. the more the distance is shorter - the better probability.
float LocalizationParticle::ProbabilityByMovement(float xDelta, float yDelta, float yawDelta)
{

	float distance = sqrt(pow(xDelta,2) + pow(yDelta,2));

	if (distance < 1)
		return 1;

	if (distance < 3)
		return 0.9;

	if (distance < 5)
		return 0.7;

	if (distance < 7)
		return 0.5;

	if (distance < 9)
		return 0.3;

	return 0.1;
}

// Get the probability of this particle by using the lidar scan.
float LocalizationParticle::ProbabilityByLidarScan(MapMatrix& graph, Robot& robot)
{
	if (graph.getNodeAtIndex(x, y)->getIsObstacle())
	{
		return 0;
	}

	float x = hamster->getPose().getX() / 0.05 + 470;
	float y = -1 * (hamster->getPose().getY() / 0.05) + 470;

	float deltaX = x - this->x;
	float deltaY = y - this->y;

	float distance = sqrt((deltaX * deltaX) + (deltaY * deltaY));

	return distance;

#if 0
	float totalHits = 0;
	float correctHits = 0;
	LidarScan lidarScan = robot.getHamster()->getLidarScan();
	// Measures the position of the
	for (unsigned int index = 200; index < 400; index++)
	{
		double distance = lidarScan.getDistance(index) * 10;
		// Takes the distance of the indexed obstacle from the lidar scanner (and from the robot)
		// If the scanner cannot see an obstacle
		if (distance >=40)
		{
			// let's move to the next sample
			continue;
		}
		totalHits++;
		const double RANGE = LIDAR_SCOPE;
		const double RATIO = RANGE / LIDAR_COUNT;
		double StartingDegree = (180 - (360 - RANGE)) / 2 + 180;
		double indexDegree = fmod(StartingDegree + ((double(index) * RATIO)) + 360, 360);
		double obstacleRadian =
				AngleUtils::convertDegreesToRadians(fmod(indexDegree + robot.GetDeltaYaw() + (180 - (360 - RANGE)) / 2 + 360, 360));
		double obstacleX = distance * cos(obstacleRadian) + x;
		obstacleX = 2*x - obstacleX;
		double obstacleY = distance * sin(obstacleRadian) + y;
		// Check if we missed boundaries.
		if ((obstacleX) < 0 || (obstacleX) >= graph.getWidth() ||
			 obstacleY < 0 || (obstacleY) >= graph.getHeight())
		{
			continue;
		}
		// Check if there's a hit on an obstacle.
		if (graph.IsCellObstacle(obstacleY, obstacleX))
		{
			correctHits++;
		}
	}
	if (totalHits == 0)
		return 0;
	else
		return correctHits/totalHits;
#endif
}

LocalizationParticle::LocalizationParticle(int i, int j, double x, double y, double yaw, double belief)
{
	this->row = i;
	this->col = j;
	this->x = x;
	this->y = y;
	this->yaw = yaw;
	this->belief = belief;
}

LocalizationParticle::LocalizationParticle(double x, double y, double yaw, double belief)
{
	this->row = 0;
	this->col = 0;
	this->x = x;
	this->y = y;
	this->yaw = yaw;
	this->belief = belief;
}

LocalizationParticle::LocalizationParticle()
{
	this->row = 0;
	this->col = 0;
	this->x = 0;
	this->y = 0;
	this->yaw = 0;
	this->belief = 0;
}

LocalizationParticle::~LocalizationParticle()
{


}
