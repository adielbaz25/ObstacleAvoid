#include "MapCell.h"
#include "stdlib.h"

MapCell::MapCell()
{
	_parent = NULL;
	setG(0);
	setH(0);
	setIsInClosedList(false);
	setIsInOpenList(false);
	setIsWaypoint(false);
}

MapCell::MapCell(double x, double y) : MapCell()
{
	_x = x;
	_y = y;
}

MapCell::~MapCell()
{
}

bool MapCell::getIsObstacle() const
{
	return _isObstacle;
}

void MapCell::setIsObstacle(bool isObstacle)
{
	_isObstacle = isObstacle;
}

bool MapCell::getIsInOpenList() const
{
	return _isInOpenList;
}

void MapCell::setIsInOpenList(bool isInOpenList)
{
	_isInOpenList = isInOpenList;
}

bool MapCell::getIsInClosedList() const
{
	return _isInClosedList;
}

void MapCell::setIsInClosedList(bool isInClosedList)
{
	_isInClosedList = isInClosedList;
}

bool MapCell::getIsWaypoint() const
{
	return _isWaypoint;
}

void MapCell::setIsWaypoint(bool isWaypoint)
{
	_isWaypoint = isWaypoint;
}

double MapCell::getG() const
{
	return _g;
}

void MapCell::setG(double g)
{
	_g = g;
}

double MapCell::getH() const
{
	return _h;
}

void MapCell::setH(double h)
{
	_h = h;
}

double MapCell::getX() const
{
	return _x;
}

void MapCell::setX(double x)
{
	_x = x;
}

double MapCell::getY() const
{
	return _y;
}

void MapCell::setY(double y)
{
	_y = y;
}

double MapCell::getF() const
{
	return _f;
}

void MapCell::setF(double f)
{
	_f = f;
}

MapCell* MapCell::getParent() const
{
	return _parent;
}

void MapCell::setParent(MapCell* parent)
{
	_parent = parent;
}


