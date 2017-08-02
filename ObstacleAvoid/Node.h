/*
 * Node.h
 *
 * Author: Adi Elbaz 206257313
 *         Yuval Ron 313584187
 */

#ifndef Node_H_
#define Node_H_

#include <cstdlib>

class Node
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
	Node* _parent;


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
	Node* getParent() const;
	void setParent(Node* parent);
	Node();
	Node(double x, double y);
	virtual ~Node();


	bool operator<(const Node& Node) const
	{
		if ((_y < Node._y) || (_y == Node._y && _x <= Node._x))
		{
			return true;
		}

		return false;
	}
};

#endif  Node_H_

