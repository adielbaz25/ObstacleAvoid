/*
 * MapPointType.h
 *
 * Author: Adi Elbaz 206257313
 *         Yuval Ron 313584187
 */
#ifndef MAPPOINTTYPE_H_
#define MAPPOINTTYPE_H_

enum MapPointType
{
	Unknown,
	Free,
	Obstacle,
	Path,
	PathStart,
	PathEnd,
	Waypoint,
	Particle,
	LidarScanObstacle,
	GoodParticle,
	BadParticle
};

#endif
