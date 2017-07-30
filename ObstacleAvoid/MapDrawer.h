#ifndef GUI_MAPDRAWER_H_
#define GUI_MAPDRAWER_H_

#include <HamsterAPIClientCPP/Hamster.h>
#include "MapPointType.cpp"
#include "MapCell.h"
#include "MapMatrix.h"
#include "LocalizationParticle.h"
#include "PositionUtils.h"
#include "AngleUtils.h"
#include "Robot.h"
#include "opencv2/imgproc.hpp"
using namespace HamsterAPI;
using namespace std;

class MapDrawer {
private:
	const string WINDOW_TITLE;
	cv::Mat* _map;
	void SetPointColor(int x, int y, int red, int green, int blue);
public:
	MapDrawer(int width, int height);

	void SetPointType(int x, int y, MapPointType mapPointType);
	void DrawMap(OccupancyGrid* occupancyGridMap, double rotationAngle);
	void DrawMapMatrix(MapMatrix* MapMatrix);
	void DrawPath(MapCell* goal);
	void DrawRobot(positionState pos, cv::Mat * map);
	void Show(positionState robotPos);
	double DrawPatricles(std::vector<LocalizationParticle *>* particles);
	cv::Mat* getMap();
	//void DrawLidarScan(std::vector<positionState> obstacles);
};

#endif /* GUI_MAPDRAWER_H_ */
