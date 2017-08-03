#ifndef STRUCTS_H_
#define STRUCTS_H_

struct rectangle
{
	unsigned startingX;
	unsigned startingY;
	unsigned endingX;
	unsigned endingY;
};

struct size
{
	double width;
	double length;
};

struct position
{
	double x;
	double y;
};

struct positionState
{
	struct position pos;
	double yaw;
};

struct parameters
{
	const char* mapFilePath;
	struct positionState startLocation;
	struct position goal;
	struct size robotSize;
	double mapResolutionCM;
	double gridResolutionCM;
};

#endif // STRUCTS_H_
