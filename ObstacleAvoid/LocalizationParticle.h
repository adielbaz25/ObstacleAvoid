#ifndef PARTICLE_H_
#define PARTICLE_H_

#include "MapMatrix.h"
#include "Robot.h"
#include <HamsterAPIClientCPP/Hamster.h>

class LocalizationManager;

// Normal number of particals to expand ~
#define NORMAL_BREED 5
// Hight number of particals to expand ~
#define HIGH_BREED 20
// The maximum number of particals which can be at the map at the
// same time. ~
#define MAX_PARTICLES_COUNT 250
// The radius we expand the particals to.+
#define EXPANSION_RADIUS 1
// The range of the angle of the child particals. +
#define YAW_RANGE 6
// The minimum belif that a partical can have.
// If the partical belif is lower than this belif then the partical
// belif is too low and it will be removed. ~
#define LOW_BELIEF_MIN 0.5
// The high belif minimum value.
// If a partical has a belif higher than this value
// it is called a high belif partical. ~
#define HIGH_BELIEF_MIN 0.85
// A number to increase the particals belif.~
#define BELIEF_NORMALIZATION_VALUE 4
// An expantion radus in the initial state (In case we dont have particals at all).
#define INITIAL_EXPANSION_RADIUS EXPANSION_RADIUS * 15
// A yaw range in the initial state (In case we dont have particals at all).
#define INITIAL_YAW_RANGE YAW_RANGE * 180
// Number of particals to expand in the initial state (In case we dont have particals at all).
#define PARTICLE_INITIAL_BREED 150
// Because the observation model returns a very
// small probebility comparing to the motion model
// we add some accuraccy in order to get the more even
#define INITIAL_OCCURACY_IN_OBSERVATION 0.4

//this class represent a particale in the map window
class LocalizationParticle {
public:
	LocalizationParticle* CreateChild(float expansionRadius, float yawRange);
	float Random(float min, float max);
	LocalizationParticle* CreateChild();
	void Update(float xDelta, float yDelta, float yawDelta, MapMatrix* map, LidarScan* lidarHandler, Robot* amnon);
	float ProbabilityByMovement(float xDelta, float yDelta, float yawDelta);
	float ProbabilityByLidarScan(MapMatrix& graph, Robot& robot);

public:
	HamsterAPI::Hamster *hamster;
	int row, col; //the row/col index in the map
	double x, y; //the exact place of the robot; x- place in column, y- place in row
	double yaw; //the heading angle of the robot
	double belief;
	LocalizationManager* manager;

	//a deafult constructor
	LocalizationParticle();
	//constructor
	LocalizationParticle(double x, double y, double yaw, double belief);
	LocalizationParticle(int i, int j, double x, double y, double yaw, double belief);
	//disconstructor
	~LocalizationParticle();
};

#endif /* PARTICLE_H_ */
