#ifndef MAP_H_
#define MAP_H_

#include <vector>
#include <HamsterAPIClientCPP/Hamster.h>
#include "MapCell.h"
#include "Constants.h"
#include "Structs.h"
#include "PositionUtils.h"

using namespace std;
using namespace HamsterAPI;

class MapMatrix
{
	private:
		vector<vector<MapCell*> > _matrix;
		rectangle getCurrentRectangle(int blowRange, unsigned currX, unsigned currY, unsigned width, unsigned height);

	public:
		int getWidth() const;
		int getHeight() const;
		MapCell* getMapCellAtIndex(int x, int y) const;
		void loadMap(cv::Mat* ogrid);
		void loadBlowMap(cv::Mat* ogrid);
		bool isAreaAnObstacle(int colIndex, int rowIndex, int resolution) const;
		MapMatrix();
		virtual ~MapMatrix();
};

#endif  MAP_H_