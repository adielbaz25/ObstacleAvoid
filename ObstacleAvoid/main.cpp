/*
 * main.cpp
 *
 * Author: Adi Elbaz 206257313
 *         Yuval Ron 313584187
 */

#include <HamsterAPIClientCPP/Hamster.h>
#include <iostream>
#include "MapMatrix.h"
#include "PathPlanner.h"
#include <stdlib.h>
#include "MapDrawer.h"
#include "LocalizationManager.h"
#include "Structs.h"
#include "Robot.h"
#include "MovementManager.h"
#include "PositionUtils.h"
#include "math.h"

using namespace std;
using namespace HamsterAPI;
HamsterAPI::Hamster * hamster;

void startRobotAction();

int main() {
	try {
		startRobotAction();
	} catch (const HamsterAPI::HamsterError & connection_error) {
		HamsterAPI::Log::i("Client", connection_error.what());
	}
	return 0;
}

bool isWaypointReached(const positionState& currLocation,
		const Node& hamsterWaypoint) {
	double distanceFromWaypoint = sqrt(
			pow(currLocation.pos.x - hamsterWaypoint.getX(), 2)
					+ pow(currLocation.pos.y - hamsterWaypoint.getY(), 2));

	return distanceFromWaypoint <= DISTANCE_FROM_WAYPOINT_TOLERANCE;
}

void initializeParticalesOnRobot(OccupancyGrid ogrid,
		MapMatrix roomBlownMap, LocalizationManager* localizationManager,
		MapDrawer* mapDrawer, Node* DestPos) {
	cout << "Initialize particales on robot" << endl;
	double bestParticalesAvrageBelief = 0;
	while (bestParticalesAvrageBelief < TARGET_START_BELIEF) {
		localizationManager->moveParticales(0, 0, 0);
		mapDrawer->DrawMap(&ogrid, MAP_ROTATION);
		mapDrawer->DrawMapMatrix(&roomBlownMap);
		mapDrawer->DrawPath(DestPos);
		bestParticalesAvrageBelief = mapDrawer->DrawPatricles(
				localizationManager->getParticles());
		positionState a;
		mapDrawer->Show(a);
		cout << "Target belief is: " << TARGET_START_BELIEF
				<< " current average belief: " << bestParticalesAvrageBelief
				<< endl;
	}
}

void startRobotAction() {

 cv::namedWindow("Room-Map");
 hamster = new HamsterAPI::Hamster(1);
 sleep(3);

 // Initialize the grid from the robot
 OccupancyGrid ogrid = hamster->getSLAMMap();

 // Initialize the MapDrawer
 MapDrawer* mapDrawer = new MapDrawer(ogrid.getWidth(), ogrid.getHeight());
 mapDrawer->DrawMap(&ogrid, 0);
 cv::Mat* drawedMap = new cv::Mat(ogrid.getWidth(), ogrid.getHeight(),CV_8UC3);
 rotateMapOnOrigin(mapDrawer->getMap(), drawedMap, MAP_ROTATION);

 // Initialize the blowed map logically
 MapMatrix roomBlownMap;
 roomBlownMap.loadBlowMap(drawedMap);

 // Initialize the robot start and destination positions nodes
 Node *startPos = roomBlownMap.getNodeAtIndex(ROBOT_START_X, ROBOT_START_Y);
 Node *DestPos = roomBlownMap.getNodeAtIndex(GOAL_X, GOAL_Y);

 // Print the start and destination positions
 cout << "Is goal an obstacle: " << roomBlownMap.getNodeAtIndex(DestPos->getX(), DestPos->getY())->getIsObstacle() << endl;

 // Find the path
 PathPlanner *pathPlanner;
 pathPlanner->findShortestPath(&roomBlownMap,startPos,DestPos);

 // Mark the waypoints
 std::list<Node* > waypoints = pathPlanner->markWaypoints(startPos, DestPos);

 // Draw and rotate the map
 mapDrawer->DrawMap(&ogrid, MAP_ROTATION);

 // Draw the extended obstacles
 mapDrawer->DrawMapMatrix(&roomBlownMap);

 // Draw the path, path start, path end and waypoints
 mapDrawer->DrawPath(DestPos);

 // Get the position of the robot
 Pose robotStartPose = hamster->getPose();

 // Set the real robot position
 struct position startPosition = {.x =
		 ROBOT_START_X + robotStartPose.getX(), .y = ROBOT_START_Y -  robotStartPose.getX()};

 // Set the position start and yaw
 struct positionState startPositionState = {.pos = startPosition, .yaw = robotStartPose.getHeading()};

 // Get the resolution of the map from hamster
 double mapResolution = ogrid.getResolution();

 LocalizationManager* localizationManager = new LocalizationManager(drawedMap, hamster, mapResolution);
 Robot robot(hamster,localizationManager, ogrid.getHeight(), ogrid.getWidth(), mapResolution);

 // Initialize the particles on the map
 localizationManager->InitParticalesOnMap(&startPositionState);

 MovementManager movementManager(&robot, mapDrawer);

if(hamster->isConnected()) {
 // foreach waypoint
 for (std::list<Node*>::reverse_iterator iter = waypoints.rbegin(); iter != waypoints.rend(); ++iter)
	{
		Node* currWaypoint = *iter;
		Node hamsterWaypoint = ConvertToHamsterLocation(currWaypoint);

		robot.realLocation = robot.currBeliefedLocation = robot.GetRealHamsterLocation();

		if (isWaypointReached(robot.currBeliefedLocation, hamsterWaypoint))
		{
			cout << endl << "Reached waypoint (" << hamsterWaypoint.getX() << ", " << hamsterWaypoint.getY() << ")" << endl << endl;
		}
		else
		{
			movementManager.NavigateToWaypoint(&hamsterWaypoint);
		}
	}

 	 cout << "The Robot reached the waypoint: (" << GOAL_X << ", " << GOAL_Y << ")" << endl;
}

}
