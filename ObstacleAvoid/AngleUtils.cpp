#include <HamsterAPIClientCPP/Hamster.h>

#include "AngleUtils.h"
#include "Constants.h"

namespace AngleUtils
{
    double convertDegreesToRadians(double angleDegrees)
    {
        double angleRadians = angleDegrees * M_PI / 180;

        return angleRadians;
    }

    double convertRadiansToDegrees(double angleRadians)
    {
        double angleDegrees = angleRadians * 180 / M_PI;

        return angleDegrees;
    }

    void rotateMapOnOrigin(cv::Mat* map, double rotationAngle)
    {
        cv::Point2f center(ROBOT_START_X,ROBOT_START_Y);
        cv::Mat rotated = cv::getRotationMatrix2D(center, rotationAngle, 1.0);
        cv::warpAffine(*map, *map, rotated, map->size());
    }
}
