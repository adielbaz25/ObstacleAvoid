#include "LocalizationManager.h"
#include <iostream>
#include <algorithm>
#include "Constants.h"

using namespace std;


void LocalizationManager::Update(float deltaX, float deltaY, float deltaYaw, LidarScan* lidarHandler, MapMatrix* map)
{
	vector<LocalizationParticle*> childsToAdd;
	vector<int> childsToRemove;
	int particlesSize = particles.size();

	// Update all the particles
	for (int i = 0; i < particlesSize; i++)
	{
		LocalizationParticle* particle = particles[i];

		particle->Update(deltaX, deltaY, deltaYaw, map, lidarHandler, robot);

		float belif = particle->belief;

		// If belief is too low - remove the particle
		if (belif <= LOW_BELIEF_MIN)
		{
			childsToRemove.push_back(i);
		}
		// If belief is high - high breed
		else if (belif >= HIGH_BELIEF_MIN &&
				 ((particlesSize + HIGH_BREED + childsToAdd.size()) < MAX_PARTICLES_COUNT))
		{
			DuplicateParticle(particle, HIGH_BREED, childsToAdd);
		}
		// If belief is normal - normal breed
		else if ((particlesSize + NORMAL_BREED + childsToAdd.size()) < MAX_PARTICLES_COUNT)
		{
			DuplicateParticle(particle, NORMAL_BREED, childsToAdd);
		}
	}

	// Removing the useless particles.
	if (childsToRemove.size() > 0)
	{
		for(int i = childsToRemove.size() - 1; i >=0 ; i--)
		{
			int indexToRemove = childsToRemove[i];
			particles.erase(particles.begin() + indexToRemove);
		}
	}

	// Adding the new particles.
	if (childsToAdd.size() > 0)
	{
		ChildsToParticles(childsToAdd);
	}
}


LocalizationParticle* LocalizationManager::BestParticle(LidarScan* lidar, float x, float y)
{
	float X = x;
	float Y = y;
	float Yaw = 0;

	// If there are no particles - we need to generate new ones
		// (occures in case we just started the robot)
	particles.clear();
	if (particles.empty())
	{
		// Create a lot of particals around the robot location
		CreateParticle(X, Y, Yaw, 1, INITIAL_EXPANSION_RADIUS, INITIAL_YAW_RANGE,  PARTICLE_INITIAL_BREED);

		LocalizationParticle* randomParticle = particles[rand() % particles.size()];

		X = randomParticle->x;
		Y = randomParticle->y;
		Yaw = randomParticle->yaw;

		// Calculate the particles belief only by observation model
		for (int i = 1; i < particles.size(); i++)
		{
			particles[i]->belief = particles[i]->ProbabilityByLidarScan(*map, *robot) * BELIEF_NORMALIZATION_VALUE;
		}

	}

	LocalizationParticle* bestParticle = particles[0];

	cout << "** Choosing best particle **" << endl;

	// Search for the best particale by it;s belief.
	for (int i = 1; i < particles.size(); i++)
	{

		if (particles[i]->belief > bestParticle->belief)
		{
			bestParticle = particles[i];
		}
	}

	X = bestParticle->x;
	Y = bestParticle->y;
	Yaw = bestParticle->yaw;

	return bestParticle;
}

bool LocalizationManager::CreateParticle(float xDelta, float yDelta, float yawDelta, float belief)
{
	return CreateParticle(xDelta, yDelta, yawDelta, belief, EXPANSION_RADIUS, YAW_RANGE, HIGH_BREED);
}

bool LocalizationManager::CreateParticle(float xDelta, float yDelta, float yawDelta, float belief, float expansionRadius, float yawRange, int childsCount)
{
	if (particles.size() + childsCount < MAX_PARTICLES_COUNT)
	{
		LocalizationParticle* particle = new LocalizationParticle();
		particle->hamster = this->hamster;
		particle->manager = this;
		particle->x = xDelta;
		particle->y = yDelta;
		particle->yaw = yawDelta;
		particle->belief = belief;

		particles.push_back(particle);
		vector<LocalizationParticle*> childs;
		DuplicateParticle(particle, childsCount, expansionRadius, yawRange, childs);
		ChildsToParticles(childs);

		return true;
	}

	return false;
}

// Create new children-particles, by using the best particles.
void LocalizationManager::DuplicateParticle(LocalizationParticle* particle, int childCount, vector<LocalizationParticle*>& childs) {

	// In case we havn't reached the maximum particles allowed number, create a new particle.
	if (particles.size() + childCount < MAX_PARTICLES_COUNT)
	{
		// Create new child-particles, and push them into the vector.
		for (int i = 0; i < childCount; i++)
		{
			LocalizationParticle* child = particle->CreateChild();
			childs.push_back(child);
		}
	}
}

// Create new children-particles, by using the best particles.
void LocalizationManager::DuplicateParticle(LocalizationParticle* particle, int childCount, float expansionRadius, float yawRange, vector<LocalizationParticle*>& childs)
{
	// In case we havn't reached the maximum particles allowed number, create a new particle.
	if (particles.size() + childCount < MAX_PARTICLES_COUNT)
	{
		// Create new child-particles, and push them into the vector.
		for (int i = 0; i < childCount; i++)
		{
			LocalizationParticle* child = particle->CreateChild(expansionRadius, yawRange);
			childs.push_back(child);
		}
	}
}

void LocalizationManager::ChildsToParticles(vector<LocalizationParticle*> childs)
{
	for (size_t i = 0; i < childs.size(); i++)
	{
		particles.push_back(childs[i]);
	}
}

LocalizationManager::LocalizationManager(OccupancyGrid *ogrid, Hamster *hamster, Robot* amnon, MapMatrix* map)
{
	this->robot = amnon;
	this->hamster = hamster;
	this->ogrid = ogrid;
	this->map = map;
}

double LocalizationManager::computeBelief(LocalizationParticle *particle, LidarScan& scan)
{
	int hitsCount = 0;
	int faultCount = 0;
	int counter = 0;

	// Running over all the scans
	while(counter < scan.getScanSize())
	{
		// Getting the current angle by multiplying the der2rad and counter
		double CurrAngle = scan.getScanAngleIncrement() * counter * DEG2RAD;

		// Comparing the distance by the scanner
		if (scan.getDistance(counter) < scan.getMaxRange() - 0.001)
		{
			double obsX = particle->x + scan.getDistance(counter) * sin(CurrAngle + particle->yaw * DEG2RAD - 180 * DEG2RAD);
			double obsY = particle->y + scan.getDistance(counter) * cos(CurrAngle + particle->yaw * DEG2RAD - 180 * DEG2RAD);

			int pixelsi = ((double) ogrid->getWidth() / 2) + (obsX);
			int pixelsj = ((double) ogrid->getHeight() / 2) - (obsY);

			// Checking where the thraw was corrent according to the grid
			if (ogrid->getCell(pixelsi, pixelsj) == CELL_OCCUPIED) {
				hitsCount++;
			} else {
				faultCount++;
			}
		}

		counter++;
	}
	return (float) hitsCount / (hitsCount + faultCount);
}

LocalizationManager::~LocalizationManager()
{
}
