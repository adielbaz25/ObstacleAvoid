/*
 * MapMatrix.cpp
 *
 * Author: Adi Elbaz 206257313
 *         Yuval Ron 313584187
 */

#ifndef MAP_H_
#define MAP_H_

#include <vector>
#include <HamsterAPIClientCPP/Hamster.h>
#include "Node.h"
#include "Constants.h"
#include "Structs.h"
#include "PositionUtils.h"

using namespace std;
using namespace HamsterAPI;

class MapMatrix
{
	private:
		vector<vector<Node*> > _matrix;
		rectangle getObstacleNeighbors(int blowRange, unsigned currX, unsigned currY, unsigned width, unsigned height);

	public:
		int getWidth() const;
		int getHeight() const;
		Node* getNodeAtIndex(int x, int y) const;
		void loadMap(cv::Mat* ogrid);
		void loadBlowMap(cv::Mat* map);
		bool isAreaAnObstacle(int colIndex, int rowIndex, int resolution) const;
		MapMatrix();
		virtual ~MapMatrix();
};

#endif  MAP_H_
