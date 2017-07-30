#ifndef MapCell_H_
#define MapCell_H_

#include <cstdlib>

class MapCell
{
	bool _isObstacle;
	bool _isInOpenList;
	bool _isInClosedList;
	bool _isWaypoint;
	double _g;
	double _h;
	double _f;
	double _x;
	double _y;
	MapCell* _parent;


public:
	bool getIsObstacle() const;
	void setIsObstacle(bool isObstacle);
	bool getIsInOpenList() const;
	void setIsInOpenList(bool isInOpenList);
	bool getIsInClosedList() const;
	void setIsInClosedList(bool isInClosedList);
	bool getIsWaypoint() const;
	void setIsWaypoint(bool isWaypoint);
	double getG() const;
	void setG(double g);
	double getH() const;
	void setH(double h);
	double getX() const;
	void setX(double x);
	double getY() const;
	void setY(double y);
	double getF() const;
	void setF(double f);
	MapCell* getParent() const;
	void setParent(MapCell* parent);
	MapCell();
	MapCell(double x, double y);
	virtual ~MapCell();


	bool operator<(const MapCell& MapCell) const
	{
		if ((_y < MapCell._y) || (_y == MapCell._y && _x <= MapCell._x))
		{
			return true;
		}

		return false;
	}
};

#endif  MapCell_H_

