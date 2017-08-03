#ifndef ANGLEUTILS_H_
#define ANGLEUTILS_H_

#include <math.h>
#include <HamsterAPIClientCPP/Hamster.h>
using namespace HamsterAPI;

namespace AngleUtils
{
    double convertDegreesToRadians(double angleDegrees);
    double convertRadiansToDegrees(double angleRadians);
    void rotateMapOnOrigin(cv::Mat* map, double rotationAngle);
}

#endif  // ANGLEUTILS_H_
