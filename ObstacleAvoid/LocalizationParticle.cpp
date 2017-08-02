/*
 * LocalizationPracticle.cpp
 *
 * Author: Adi Elbaz 206257313
 *         Yuval Ron 313584187
 */

#include "LocalizationParticle.h"

#include <HamsterAPIClientCPP/Hamster.h>
#include <iostream>


using namespace std;

LocalizationParticle::LocalizationParticle(int i, int j, double x, double y, double yaw, double belief)
{
	this->row = i;
	this->col = j;
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
