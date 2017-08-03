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
		int calculateBlowRange(double robotSizeCm, double resolutionCm);
		rectangle getCurrentRectangle(int blowRange, unsigned currX, unsigned currY, unsigned width, unsigned height);

	public:
		int getWidth() const;
		int getHeight() const;
		Node* getNodeAtIndex(int x, int y) const;
		void loadMap(cv::Mat* roomRealMapFromMemory);
		void loadBlowMap(cv::Mat* roomRealMapFromMemory);
		MapMatrix getBlownMapMatrix();
		void resizeMap(int pixelsPerCell, MapMatrix* output) const;
		bool isAreaAnObstacle(int colIndex, int rowIndex, int resolution) const;
		void colorArea(unsigned width, unsigned height,
				struct position pos, int r, int g, int b);
		bool IsCellObstacle(float x, float y) const;
		MapMatrix();
		virtual ~MapMatrix();
};

#endif // MAP_H_
