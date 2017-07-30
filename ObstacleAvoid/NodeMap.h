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

class NodeMap
{
	private:
		vector<vector<Node*> > _matrix;
		int calculateBlowRange(double robotSizeCm, double resolutionCm);
		rectangle getCurrentRectangle(int blowRange, unsigned currX, unsigned currY, unsigned width, unsigned height);

	public:
		int getWidth() const;
		int getHeight() const;
		Node* getNodeAtIndex(int x, int y) const;
		void loadMap(cv::Mat* roomRealMapFromMemory);
		void loadBlowMap(cv::Mat* roomRealMapFromMemory);
		bool isAreaAnObstacle(int colIndex, int rowIndex, int resolution) const;
		NodeMap();
		virtual ~NodeMap();
};

#endif  MAP_H_
