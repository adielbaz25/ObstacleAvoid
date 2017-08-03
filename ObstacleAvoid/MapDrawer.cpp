#include "MapDrawer.h"
#include "LocalizationParticle.h"

MapDrawer::MapDrawer(int width, int height) : WINDOW_TITLE("Room-Map"), _goodParticlesToDraw(5)
{
	cv::namedWindow("Room-Map");
	_map = new cv::Mat(width, height,CV_8UC3,cv::Scalar(0,0,0));

}

void MapDrawer::DrawMap(const OccupancyGrid* occupancyGridMap, double rotationAngle)
{
	size_t width = occupancyGridMap->getWidth();
	size_t height = occupancyGridMap->getHeight();

	for (size_t x = 0; x < height; x++)
	{
	     for (size_t y = 0; y < width; y++)
	     {
              if (occupancyGridMap->getCell(x, y) == CELL_FREE)
              {
                  MapDrawer::SetPointType(x,y, Free);
              }
              else if (occupancyGridMap->getCell(x, y) == CELL_OCCUPIED)
              {
                  MapDrawer::SetPointType(x,y, Obstacle);
              }
              else
              {
                  MapDrawer::SetPointType(x,y, Unknown);
              }
	     }
	}

	if(rotationAngle != 0)
	{
		AngleUtils::rotateMapOnOrigin(_map, rotationAngle);
	}
}

void MapDrawer::DrawMapMatrix(MapMatrix* MapMatrix)
{
	int width = MapMatrix->getWidth();
	int height = MapMatrix->getHeight();

	for (int y = 0; y < height; y++) {
	     for (int x = 0; x < width; x++) {
	      if (MapMatrix->getNodeAtIndex(y,x)->getIsObstacle())
	      {
	    	  MapDrawer::SetPointType(x, y, Obstacle);
	      }
	   }

	}
}

void MapDrawer::DrawPath(Node* goal)
{
	MapDrawer::SetPointType(goal->getY() , goal->getX() , PathEnd);

	Node* currentNode = goal->getParent();
	while(currentNode != NULL)
	{
		Node* nextNode = currentNode->getParent();

		if(nextNode == NULL)
		{
			MapDrawer::SetPointType(currentNode->getY() ,currentNode->getX()  , PathStart);
		}
		else if(currentNode->getIsWaypoint())
		{
			MapDrawer::SetPointType(currentNode->getY() ,currentNode->getX() , Waypoint);
		}
		else
		{
			MapDrawer::SetPointType(currentNode->getY() ,currentNode->getX() , Path);
		}

		currentNode = nextNode;
	}

	MapDrawer::SetPointType(goal->getY() ,goal->getX() , PathStart);

}

void MapDrawer::DrawPatricles(std::vector<LocalizationParticle *>* particles)
{
	_goodParticlesDrawn = 0;
	std::list<LocalizationParticle*>::const_iterator iterator;
		for (int i = particles->size() - 1; i >= 0 ; i--) {
			LocalizationParticle* particale = (*particles)[i];
			if(i>particles->size() - _goodParticlesToDraw)
				MapDrawer::SetPointType(particale->x ,particale->y, GoodParticle);
			else
				MapDrawer::SetPointType(particale->x ,particale->y, BadParticle);

	printf("good particles drawn: %d\n", _goodParticlesDrawn);

	}
}

void MapDrawer::SetPointType(size_t x, size_t y, MapPointType mapPointType)
{
	switch(mapPointType)
	{
		case(Unknown) :
		{
			this->SetPointColor(x, y, 128, 128, 128);
			break;
		}
		case(Free) :
		{
			this->SetPointColor(x, y, 255, 255, 255);
			break;
		}
		case(Obstacle) :
		{
			this->SetPointColor(x, y, 0, 0, 0);
			break;
		}
		case(PathStart) : {
		    this->SetPointColor(x, y, 0, 0, 255);
			break;
		}
		case(PathEnd): {
		    this->SetPointColor(x, y, 0, 255, 0);
			break;
		}
		case(Path) :
		{
		    this->SetPointColor(x, y, 255, 0, 0);
			break;
		}
		case(Waypoint) :
		{
		    this->SetPointColor(x, y, 255, 255, 0);
			break;
		}
		case(Particle) :
		{
			if ((MapDrawer::_map->at<cv::Vec3b>(x, y)[0] == 0) &&
				(MapDrawer::_map->at<cv::Vec3b>(x, y)[1] == 0) &&
				(MapDrawer::_map->at<cv::Vec3b>(x, y)[2] == 0))
			{
				this->SetPointColor(x, y, 255, 0, 0);
			}
			else
			{
				this->SetPointColor(x, y, 0, 0, 255);
			}
			break;
		}
		case(LidarScanObstacle) :
		{
		    this->SetPointColor(x, y, 255, 0, 255);
			break;
		}
		case(GoodParticle) :
		{
			if ((MapDrawer::_map->at<cv::Vec3b>(x, y)[0] == 0) &&
				(MapDrawer::_map->at<cv::Vec3b>(x, y)[1] == 0) &&
				((MapDrawer::_map->at<cv::Vec3b>(x, y)[2] == 0) || (MapDrawer::_map->at<cv::Vec3b>(x, y)[2] == 255)))
			{
				this->SetPointColor(x, y, 255, 0, 0);
			}
			else
			{
				++_goodParticlesDrawn;
				this->SetPointColor(x, y, 0, 255, 0);
			}
			break;
		}
		case(BadParticle) :
		{
			if (_goodParticlesDrawn < _goodParticlesToDraw)
			{
				this->SetPointType(x, y, GoodParticle);
			}
			else
			{
				this->SetPointColor(x, y, 255, 0, 0);
			}

			break;
		}
	}
}

void MapDrawer::SetPointColor(size_t x, size_t y, int red, int green, int blue)
{
	if(x >= 0 && y >= 0)
	{
		MapDrawer::_map->at<cv::Vec3b>(x, y)[0] = blue;
		MapDrawer::_map->at<cv::Vec3b>(x, y)[1] = green;
		MapDrawer::_map->at<cv::Vec3b>(x, y)[2] = red;
	}
}

void MapDrawer::Show()
{
	cv::Point2f center(ROBOT_START_X,ROBOT_START_Y);
	cv::Mat resultMap;
	cv::Mat scaled = cv::getRotationMatrix2D(center, 0, 2.0);
	cv::warpAffine(*_map, resultMap, scaled, _map->size());

	cv::imshow(MapDrawer::WINDOW_TITLE, resultMap);
	cv::waitKey(100);
}

void MapDrawer::SaveCurrentMap()
{
	this->_savedMapState = this->_map->clone();
}

void MapDrawer::RevertToSavedMap()
{
	*(this->_map) = this->_savedMapState.clone();
}

cv::Mat* MapDrawer::getMap()
{
	return _map;
}
