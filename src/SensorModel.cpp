#include "SensorModel.hpp"
#include <opencv2/opencv.hpp>
#include "Profiler.hpp"

#ifdef DEBUG
#define SPDLOG_ACTIVE_LEVEL SPDLOG_LEVEL_DEBUG
#endif
#include "spdlog/spdlog.h"

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
		(std::abs(worldMap->data[y][x]) < threshold))
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
	Profiler<std::chrono::microseconds> pf("Time (microseconds) taken to perform raycasting");
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
	// iterate from 0 to 180 degrees
	for (int sweepAngle = 1; sweepAngle <=180; sweepAngle++)
	{
		double slopeAngle = sweepAngle + laserPoseInWorldFrame.theta;
		double xNew = laserPoseInWorldFrame.x;
		double yNew = laserPoseInWorldFrame.y;
		int count = 0;
		while (keepCasting(std::round(xNew), std::round(yNew), threshold, worldMap))
		{
			xNew = laserPoseInWorldFrame.x + count*stepSize*cos(slopeAngle);
			yNew = laserPoseInWorldFrame.y + count*stepSize*cos(slopeAngle);
			count++;
		}
		simulatedRayCast[sweepAngle-1] = (int)std::min(2000.0,std::round(stepSize*(count-1)));
	}
	return simulatedRayCast;
}
