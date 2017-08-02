/*
 * LOcalizationManger.cpp
 *
 * Author: Adi Elbaz 206257313
 *         Yuval Ron 313584187
 */

#include "LocalizationManager.h"
#include <iostream>
#include <algorithm>

using namespace std;



LocalizationManager::LocalizationManager(cv::Mat* map, Hamster *hamster, double mapResolution)
{
	this->hamster = hamster;
	this->map = map;
	this->mapResolution = mapResolution;
}

void LocalizationManager::createRandomParticle(LocalizationParticle *par)
{
	//Randomizing an angle
	par->yaw = rand() % 360;
	cv::Vec3b coloredPoint;

	//set random column and row while the random cell chosen isn't free
	do {
		par->col = rand() % map->cols;
		par->row = rand() % map->rows;
		coloredPoint = map->at<cv::Vec3b>(par->row, par->col);


	// While the particle isnt cell free
	} while (!(coloredPoint[0] == 255 && coloredPoint[1] == 255 && coloredPoint[2] == 255));

	//Conversion
	par->x = (par->col - ROBOT_START_X) * mapResolution;
	par->y = (ROBOT_START_Y - par->row ) * mapResolution;


}
double LocalizationManager:: randNumberFactor(int level)
{
	if(level == 3)
		return 0.4 -0.8*(double)rand()/(double)RAND_MAX;
	else if(level == 2)
		return 0.2-0.4*(double)rand()/(double)RAND_MAX;
	else
		return 0.1-0.2*(double)rand()/(double)RAND_MAX;
}

double LocalizationManager:: randNumberFactorYaw(int level)
{
	if(level == 5)
		return 180 - rand() % 360;
	else if(level == 4)
		return 90 - rand() % 180;
	else if(level == 3)
		return 30 - rand() % 60;
	else if(level == 2)
		return 10 - rand() % 20;
	else
		return 5 - rand() % 10;

}

void LocalizationManager::createNeighborParticales(LocalizationParticle *badParticale,  LocalizationParticle *goodParticale)//previous particle , new particle
{
	cv::Vec3b coloredPoint;


	do {
		if (goodParticale->belief < 0.3)
		{
			badParticale->x = goodParticale->x+ randNumberFactor(3);
			badParticale->y = goodParticale->y + randNumberFactor(3);
		}
		else if (goodParticale->belief < 0.6)
		{
			badParticale->x = goodParticale->x+ randNumberFactor(2);
			badParticale->y = goodParticale->y+ randNumberFactor(2);
		}
		else
		{
			badParticale->x = goodParticale->x+ randNumberFactor(1);
			badParticale->y = goodParticale->y+ randNumberFactor(1);
		}

		badParticale->row = (double) ROBOT_START_Y - badParticale->y / mapResolution;
		badParticale->col = badParticale->x / mapResolution+ ROBOT_START_X;

		coloredPoint = map->at<cv::Vec3b>(badParticale->row, badParticale->col);

	} while (!(coloredPoint[0] == 255 && coloredPoint[1] == 255 && coloredPoint[2] == 255));

	if (goodParticale->belief < 0.2)
		badParticale->yaw = (goodParticale->yaw + (randNumberFactorYaw(5)));
	else if (goodParticale->belief < 0.4)
		badParticale->yaw = (goodParticale->yaw + (randNumberFactorYaw(4)));
	else if (goodParticale->belief < 0.6)
		badParticale->yaw = (goodParticale->yaw + (randNumberFactorYaw(3)));
	else if (goodParticale->belief < 0.8)
		badParticale->yaw = (goodParticale->yaw + (randNumberFactorYaw(2)));
	else
		badParticale->yaw = (goodParticale->yaw + (randNumberFactorYaw(1)));

	if(badParticale->yaw >= 360)
		badParticale->yaw -= 360;
	if(badParticale->yaw < 0)
		badParticale->yaw += 360;


}




void LocalizationManager::InitParticalesOnMap(positionState * ps)
{
	particles.resize(NUM_OF_PARTICALES);
	cv::Vec3b coloredPoint;

	initSourceParticle(ps);

	for (size_t i = 0; i < particles.size() - 1 ; i++)
	{
		particles[i] = new LocalizationParticle();

		//Randomizing an angle
		double degYaw = ps->yaw*180/M_PI + 180;
		if(degYaw >= 360)
			degYaw -= 360;
		if(degYaw < 0)
			degYaw += 360;

		particles[i]->yaw = rand() % 360;

		//set random column and row while the random cell chosen isn't free
		do {
			particles[i]->col = ps->pos.x + rand() % 5 ;
			particles[i]->row = ps->pos.y + rand() % 5;

			coloredPoint = map->at<cv::Vec3b>(particles[i]->row, particles[i]->col);

		} while (!(coloredPoint[0] == 255 && coloredPoint[1] == 255 && coloredPoint[2] == 255));

		//Conversion
		particles[i]->x = (particles[i]->col - ROBOT_START_X) * mapResolution;
		particles[i]->y = (ROBOT_START_Y - particles[i]->row ) * mapResolution;

	}


}

void LocalizationManager::initSourceParticle(positionState * ps) {
	particles[particles.size() - 1] = new LocalizationParticle();
	particles[particles.size() - 1]->col = ps->pos.x;
	particles[particles.size() - 1]->row = ps->pos.y;
	particles[particles.size() - 1]->yaw = ps->yaw;
	particles[particles.size() - 1]->belief = 1;
}

double LocalizationManager::updateBelief(LocalizationParticle *p)
{
	cv::Vec3b coloredPoint;
	LidarScan scan = hamster->getLidarScan();

	int hits = 0;
	int misses = 0;

	for (unsigned int i = 0; i < scan.getScanSize(); i++)
	{
		double angle = scan.getScanAngleIncrement() * i * DEG2RAD;

		if (scan.getDistance(i) < scan.getMaxRange() - 0.001)
		{

			double wallX = p->x + scan.getDistance(i)* cos(angle + p->yaw * DEG2RAD- 180 * DEG2RAD);


			double wallY = p->y+ scan.getDistance(i)* sin(angle + p->yaw * DEG2RAD- 180 * DEG2RAD);


			int i = (double) ROBOT_START_Y - wallY / mapResolution;


			int j = wallX / mapResolution + ROBOT_START_X;


			coloredPoint = map->at<cv::Vec3b>(i, j);

			// Maybe change to cell occupied
			if (!(coloredPoint[0] == 255 && coloredPoint[1] == 255 && coloredPoint[2] == 255))
			{
				hits++;
			}
			else
			{
				misses++;
			}
		}
	}




	return (double) hits / (hits + misses);
}

bool compareParticals(LocalizationParticle* x, LocalizationParticle* y)
{
	if(x->belief < y->belief)
		return true;
	return false;
}

bool LocalizationManager::tryReturnBackOutOfRangeParticle(LocalizationParticle *p)
{
	cv::Vec3b coloredPoint;
	LocalizationParticle * copyPar = new LocalizationParticle(*p);
	int distant;
	int count = 0;
	do {
		//+-7 for distant
		distant = 14 - rand() % 28;
		p->col = copyPar->col + distant;
		distant = 14 - rand() % 28;
		p->row = copyPar->row + distant;
		count++;
		coloredPoint = map->at<cv::Vec3b>(copyPar->row, copyPar->col);

	} while (!(coloredPoint[0] == 255 && coloredPoint[1] == 255 && coloredPoint[2] == 255)
			&& count < TRY_TO_BACK);

	//Conversion
	p->x = (p->col - ROBOT_START_X) * mapResolution;
	p->y = (ROBOT_START_Y - p->row ) * mapResolution;


	delete copyPar;

	return count < TRY_TO_BACK;
}

void LocalizationManager::calculateYaw(LocalizationParticle* p, double deltaYaw) {
	p->yaw += deltaYaw;
	if (p->yaw >= 360) {
		p->yaw -= 360;
	}
	if (p->yaw < 0) {
		p->yaw += 360;
	}
}

void LocalizationManager::calculateRealPos(LocalizationParticle* p,
										   double deltaX,
										   double deltaY,
										   double deltaYaw) {
	double distance = sqrt(deltaX * deltaX + deltaY * deltaY);
	p->x += distance * cos(p->yaw * DEG2RAD);
	p->y += distance * sin(p->yaw * DEG2RAD);

	calculateYaw(p, deltaYaw);
}

void LocalizationManager::calculatePositionOnMap(LocalizationParticle* p) {
	p->row = (double) ROBOT_START_Y - p->y / mapResolution;
	p->col = p->x / mapResolution + ROBOT_START_X;


}

void LocalizationManager::replaceBadOutOfRangeParticle(LocalizationParticle* p, int size) {
	int indexFromTop = size - rand() % TOP_PARTICALES - 1;

	if (particles[indexFromTop]->belief > 0.4)
	{
		createNeighborParticales(p, particles[indexFromTop]);
	}
	else
	{
		createRandomParticle(p);
	}
}

void LocalizationManager:: moveParticales(double deltaX, double deltaY, double deltaYaw)
{
	cv::Vec3b coloredPoint;
	int size = particles.size();
	for (size_t i = 0; i < particles.size(); i++)
	{
		LocalizationParticle *p = particles[i];

		calculateRealPos(p, deltaX, deltaY, deltaYaw);
		calculatePositionOnMap(p);

		coloredPoint = map->at<cv::Vec3b>(p->row, p->col);


		if (!(coloredPoint[0] == 255 && coloredPoint[1] == 255 && coloredPoint[2] == 255) &&
			(p->belief <= MIN_BELIEF || tryReturnBackOutOfRangeParticle(p)))
		{
			replaceBadOutOfRangeParticle(p, size);
		}

		p->belief = updateBelief(p);
	}

	std::sort(particles.begin(), particles.end(), compareParticals);

	for (int i = 1; i <= BAD_PARTICALES; i++)
	{
		if (particles[size - i]->belief > MIN_BELIEF)
		{
			createNeighborParticales(particles[i - 1], particles[size - i]);
			updateBelief(particles[i - 1]);
		}
		else
		{
			createRandomParticle(particles[i - 1]);
			updateBelief(particles[i - 1]);
		}
	}

}

void LocalizationManager::printParticles()
{
	for (unsigned int i = 0; i < particles.size(); i++)
	{

		cout << "Particle's Number " << i <<": " << endl;
		cout<< "x : "<<particles[i]->x <<endl;
		cout<< "y : "<< particles[i]->y<<endl;
		cout<< "heading angle : " << particles[i]->yaw <<endl;
		cout<<"belief : "<< particles[i]->belief << endl<<endl<<endl;
	}
}

vector<LocalizationParticle *>* LocalizationManager::getParticles()
{
	return &particles;
}

positionState LocalizationManager::getPosition() {
	LocalizationParticle* localizationParticle = particles[particles.size() -1];

	positionState positionState;
	positionState.pos.x = localizationParticle->col;
	positionState.pos.y = localizationParticle->row;
	positionState.yaw = localizationParticle->yaw;

	return positionState;
}

double LocalizationManager::getBestBelief() {
	return particles[particles.size() -1]->belief;
}

LocalizationManager::~LocalizationManager()
{
}

