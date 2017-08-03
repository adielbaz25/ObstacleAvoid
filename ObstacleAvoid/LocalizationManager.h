#ifndef LOCALIZATIONMANAGER_H_
#define LOCALIZATIONMANAGER_H_

#include "LocalizationParticle.h"
#include <vector>
#include <HamsterAPIClientCPP/Hamster.h>
#include "Constants.h"
#include "Structs.h"
#include "MapMatrix.h"
#define NUM_OF_PARTICALES 350
#define TRY_TO_BACK 20
#define TOP_PARTICALES 80
#define BAD_PARTICALES 80


using namespace std;
using namespace HamsterAPI;

//this class manage all the particals in the map
class LocalizationManager
{
public:
	void Update(float deltaX, float deltaY, float deltaYaw, LidarScan* lidarHandler, MapMatrix* map);
	LocalizationParticle* BestParticle(LidarScan* lidar, float x, float y);
	bool CreateParticle(float xDelta, float yDelta, float yawDelta, float belief);
	bool CreateParticle(float xDelta, float yDelta, float yawDelta, float belief, float expansionRadius, float yawRange, int childsCount);
	void DuplicateParticle(LocalizationParticle* particle, int childCount, vector<LocalizationParticle*>& childs);
	void DuplicateParticle(LocalizationParticle* particle, int childCount, float expansionRadius, float yawRange, vector<LocalizationParticle*>& childs);
	void ChildsToParticles(vector<LocalizationParticle*> childs);

	vector<LocalizationParticle *> particles;

public:
	MapMatrix* map;
	Robot* robot;
	Hamster *hamster;

	//return back the particales which out of the free cells range to free cells range
	bool tryReturnBackOutOfRangeParticle(LocalizationParticle *particle);

	//update the particale's belief
	double updateBelief(LocalizationParticle *particle);

	//create a random particale
	void createRandomParticle(LocalizationParticle *particle);

	//close the bad particale near to a particale with a high belief
	void createNeighborParticales(LocalizationParticle *prevP,  LocalizationParticle *newP);

	void initSourceParticle(positionState * ps);

	double randNumberFactor(int level);
	double randNumberFactorYaw(int level);
	void calculateRealPos(LocalizationParticle* p,
						  double deltaX,
						  double deltaY,
						  double deltaYaw);
	void calculateYaw(LocalizationParticle* p, double deltaYaw);
	void calculatePositionOnMap(LocalizationParticle* p);
	void replaceBadOutOfRangeParticle(LocalizationParticle* p, int size);
	double computeBelief(LocalizationParticle *particle, LidarScan& scan);

public:

	OccupancyGrid *ogrid;

	//constructor
	LocalizationManager( OccupancyGrid *ogrid, Hamster *hamster, Robot* amnon, MapMatrix* map);

	//getter
	vector<LocalizationParticle *>* getParticles();

	//print the particale's vector
	void printParticles() ;

	//create new random particals on the map
	void InitParticalesOnMap(positionState * ps);

	//move the particales according to the robot's movement
	void moveParticales(double deltaX, double deltaY, double deltaYaw);

	positionState getPosition();
	double getBestBelief();

	//distructor
	virtual ~LocalizationManager();
};

#endif /* LOCALIZATIONMANAGER_H_ */
