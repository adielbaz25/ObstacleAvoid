#include "MapMatrix.h"

MapMatrix::MapMatrix()
{
}

MapMatrix::~MapMatrix()
{
	// TODO Iterate over the 2D vector and deallocate every Node reference
}

int MapMatrix::getWidth() const
{
	return _matrix.size();
}

int MapMatrix::getHeight() const
{
	return _matrix[0].size();
}

Node* MapMatrix::getNodeAtIndex(int x, int y) const
{
	return _matrix[y][x];
}

void MapMatrix::loadMap(cv::Mat* map)
{
	unsigned mapWidth = map->cols;
	unsigned mapHeight = map->rows;

	vector<vector<Node*> > matrix(mapWidth, vector<Node*>(mapHeight));

	for (unsigned y = 0; y < mapHeight; y++)
	{
		for (unsigned x = 0; x < mapWidth; x++) {
			matrix[y][x] = new Node(x, y);

			cv::Vec3b coloredPoint = map->at<cv::Vec3b>(y, x);
			matrix[y][x]->setIsObstacle(coloredPoint[0] == 0 && coloredPoint[1] == 0 && coloredPoint[2] == 0);
		}
	}
	_matrix = matrix;

}

// Creates a new image from the source file where every obstacle is blown up

void MapMatrix::loadBlowMap(cv::Mat* map)
{
	loadMap(map);

	// Holds the original map
	struct rectangle currRec;
	double robotSizeCm = ROBOT_SIZE;
	double resolutionCm = RESOLUTION_SIZE;

	unsigned mapWidth = map->cols;
	unsigned mapHeight = map->rows;

	std::list<Node*> obstacles;

	// Expand size is half of the robot size
	robotSizeCm /= 2;

	// Blow the map
	robotSizeCm *= BLOW_ROBOT_FACTOR;

	// Calculates the expand range
	int blowRange = ceil(robotSizeCm / resolutionCm);

	// Looping to scan the original map
	for (unsigned y = 0; y < mapHeight; y++)
	{
		for (unsigned x = 0; x < mapWidth; x++)
		{
			// Checks if the original Node is obstacle
			if (_matrix[y][x]->getIsObstacle())
			{
				obstacles.push_front(_matrix[y][x]);
			}
		}
	}

	std::list<Node*>::const_iterator iterator;
	for (iterator = obstacles.begin(); iterator != obstacles.end(); ++iterator) {

			// Calculates a rectangle to set as obstacles around the current Node
					currRec = getCurrentRectangle(blowRange, (*iterator)->getX(), (*iterator)->getY(),
							mapWidth, mapHeight);

					// Loops to set the neighbors in the blow range as obstacles
					for (unsigned neighborY = currRec.startingY; neighborY < currRec.endingY; neighborY++)
					{
						for (unsigned neighborX = currRec.startingX; neighborX < currRec.endingX; neighborX++)
						{
							// Sets the current neighbor Node as obstacle obstacle
							_matrix[neighborY][neighborX]->setIsObstacle(true);
						}
					}
	}

}

bool MapMatrix::isAreaAnObstacle(int colIndex, int rowIndex, int resolution) const
{
	for (int i = colIndex * resolution; i < (colIndex * resolution) + resolution; i++)
	{
		for (int j = rowIndex * resolution; j < (rowIndex * resolution) + resolution; j++)
		{
			if (getNodeAtIndex(i, j)->getIsObstacle())
			{
				return true;
			}
		}
	}

	return false;
}

// Calculates a rectangle that wraps the current index
rectangle MapMatrix::getCurrentRectangle(int blowRange, unsigned currX, unsigned currY, unsigned width, unsigned height)
{
	struct rectangle result;

	// Gets the leftmost upmost index
	int startingY = currY - blowRange;

	// Checks if out of bounds
	if (startingY < 0)
	{
		startingY = 0;
	}

	int startingX = currX - blowRange;

	// Checks if out of bounds
	if (startingX < 0)
	{
		startingX = 0;
	}

	// Gets the rightmost downmost index
	unsigned endingY = currY + blowRange;

	// Checks if out of bounds
	if (endingY > height)
	{
		endingY = height;
	}

	unsigned endingX = currX + blowRange;

	// Checks if out of bounds
	if (endingX > width)
	{
		endingX = width;
	}

	// Setting the result
	result.startingX = startingX;
	result.startingY = startingY;
	result.endingX = endingX;
	result.endingY = endingY;

	return result;
}

