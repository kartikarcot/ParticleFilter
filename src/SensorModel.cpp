#include <opencv2/opencv.hpp>
#include "SensorModel.hpp"
#include "Profiler.hpp"
#include "Map.hpp"

#ifdef DEBUG
#define SPDLOG_ACTIVE_LEVEL SPDLOG_LEVEL_DEBUG
#endif
#include "spdlog/spdlog.h"

#define PI 3.14159
#define TO_RADIANS(x) (PI*x/180.0)


inline bool keepCasting(
		const int &x,
		const int &y,
		const double &threshold,
		const std::shared_ptr<Map> &worldMap)
{
	if ((x >= 0) &&
		(x < worldMap->maxX) && 
		(y >= 0) && 
		(y < worldMap->maxY) &&
		(worldMap->data[y][x] >= 0) &&
		(worldMap->data[y][x] < threshold))
	{
		return true;
	}
	return false;
}

std::vector<int> SensorModel::rayCasting(
					const Pose2D &laserPoseInOdomFrame,
					const Pose2D &robotPoseInOdomFrame,
					const Pose2D &particlePoseInWorldFrame,
					const std::shared_ptr<Map> &worldMap)
{
	// get the laser pose in world frame for the particle belief
	double deltaX = laserPoseInOdomFrame.x - robotPoseInOdomFrame.x;
	double deltaY = laserPoseInOdomFrame.y - robotPoseInOdomFrame.y;
	double deltaTheta = laserPoseInOdomFrame.theta - robotPoseInOdomFrame.theta;
	Pose2D laserPoseInWorldFrame(
			particlePoseInWorldFrame.x + deltaX,
			particlePoseInWorldFrame.y +deltaY,
			particlePoseInWorldFrame.theta + deltaTheta);

	// perform ray casting from the laserPoseInWorldFrame
	std::vector<int> simulatedRayCast(180,INT_MAX);
	std::vector<Pose2D> rayPoints;
	// iterate from 0 to 180 degrees

	// SPDLOG_DEBUG("ORIGIN ({},{}): {}", 
	//		std::round(laserPoseInWorldFrame.x), 
	//		std::round(laserPoseInWorldFrame.y), 
	//		worldMap->data[std::round(laserPoseInWorldFrame.y)][std::round(laserPoseInWorldFrame.x)]);

	for (int sweepAngle = 1; sweepAngle <=180; sweepAngle++)
	{
		double slopeAngle = TO_RADIANS(sweepAngle) + laserPoseInWorldFrame.theta;
		double xNew = laserPoseInWorldFrame.x;
		double yNew = laserPoseInWorldFrame.y;
		int count = 0;
		while (keepCasting(std::round(xNew), std::round(yNew), 1, worldMap))
		{
			xNew = laserPoseInWorldFrame.x + count*stepSize*cos(slopeAngle);
			yNew = laserPoseInWorldFrame.y + count*stepSize*sin(slopeAngle);
			rayPoints.push_back(Pose2D(xNew, yNew, 0));
			count++;
		}
		simulatedRayCast[sweepAngle-1] = (int)std::min(2000.0,std::round(stepSize*(count-1)));
	}
	rayPoints.push_back(laserPoseInWorldFrame);
	#ifdef DEBUG
		visualizeMap(worldMap, rayPoints, "Raycast visualisation");
	#endif
	return simulatedRayCast;
}
