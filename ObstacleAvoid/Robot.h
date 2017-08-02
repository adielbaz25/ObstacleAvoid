/*
 * Robot.h
 *
 * Author: Adi Elbaz 206257313
 *         Yuval Ron 313584187
 */

#ifndef ROBOT_H_
#define ROBOT_H_

#include <HamsterAPIClientCPP/Hamster.h>
#include <set>
#include <math.h>
#include "Node.h"
#include "MapMatrix.h"
#include "Structs.h"
#include "MapDrawer.h"
#include "Constants.h"
#include "AngleUtils.h"
#include "LocalizationManager.h"

using namespace HamsterAPI;
using namespace std;

class Robot {
private:
	Hamster * hamster;
	double hamsterStartX, hamsterStartY;
	LocalizationManager * localizationManager;
	double resolution;
	double mapHeight, mapWidth;

public:
	positionState prevBeliefedLocation, currBeliefedLocation, realLocation;

	Robot(Hamster * hamster, LocalizationManager * localizationManager, double mapHeight, double mapWidth, double resolution);
	positionState GetRealHamsterLocation();
	void updateHamsterRealLocation();
	void setHamsterSpeed(double speed, double wheelsAngle);
	virtual ~Robot();
};

#endif /* ROBOT_H_ */
