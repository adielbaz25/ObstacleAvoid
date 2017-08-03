#ifndef MAPDRAWER_H_
#define MAPDRAWER_H_

#include <HamsterAPIClientCPP/Hamster.h>
#include "MapPointType.h"
#include "Node.h"
#include "MapMatrix.h"
#include "PositionUtils.h"
#include "AngleUtils.h"
#include "opencv2/imgproc.hpp"
using namespace HamsterAPI;
using namespace std;

class LocalizationParticle;

class MapDrawer
{
public:
	MapDrawer(int width, int height);

	void SetPointType(size_t x, size_t y, MapPointType mapPointType);
	void DrawMap(const OccupancyGrid* occupancyGridMap, double rotationAngle);
	void DrawMapMatrix(MapMatrix* MapMatrix);
	void DrawPath(Node* goal);
	void Show();
	void SaveCurrentMap();
	void RevertToSavedMap();
	void DrawPatricles(std::vector<LocalizationParticle *>* particles);
	cv::Mat* getMap();

private:
    const string WINDOW_TITLE;
    cv::Mat* _map;
    cv::Mat _savedMapState;
    const size_t _goodParticlesToDraw;
    size_t _goodParticlesDrawn;
    void SetPointColor(size_t x, size_t y, int red, int green, int blue);
};

#endif
