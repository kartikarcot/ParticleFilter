
#include <iostream>
#include <memory>
#include <Map.hpp>
#include <LogReader.hpp>
#include <ParticleFilter.hpp>
#include <MotionModel.hpp>
#include <boost/optional.hpp>
#include "Profiler.hpp"
#include "config.hpp"
#include <Utils.hpp>

#ifdef DEBUG
#define SPDLOG_ACTIVE_LEVEL SPDLOG_LEVEL_DEBUG
#endif
#include "spdlog/spdlog.h"

int main(int argc, char **argv)
{
	spdlog::set_level(
        static_cast<spdlog::level::level_enum>(SPDLOG_ACTIVE_LEVEL));

	spdlog::set_pattern("%^[%l] [%s]%$ %v");

	if (argc!=3)
	{
		SPDLOG_ERROR("Invalid number of arguments\n.  \
				Follow this format <path to exec> <path to map> <path to log>");
		return 1;
	}

	std::shared_ptr<Map> worldMap = makeMap(std::string(argv[1]));
	LogReader logReader((std::string(argv[2])));
	boost::optional<Log> log;

	ParticleFilter particleFilter = ParticleFilter(1 , worldMap);
	particleFilter.particles[0] = Pose2D(4000,4000,90);
	std::vector<double> alphas = ALPHAS;
	MotionModel motionModel(ROT1_VAR, TRANS_VAR, ROT2_VAR, alphas);
	
	bool firstTime = true;
	Pose2D odomPreviousMeasure, odomCurrentMeasure;
	Pose2D particlePreviousMeasure, particleCurrentMeasure;
	std::vector<cv::Point2d> trajectory;
	
	while((log = logReader.getLog()))
	{
		SPDLOG_DEBUG("The log read was {} {} {}, LogType {}", 
				log->robotPose.x, log->robotPose.y, log->robotPose.theta, log->logType);
		Profiler<std::chrono::milliseconds> pf("Time (ms) taken to perform one iteration of Particle Filter");
		// if it is the first log, then copy into odomPrevious and continue
		if (firstTime)
		{
			odomPreviousMeasure = log->robotPose;
			firstTime = false;
			continue;
		}

		// set current odom measure to odom robot pose read from log
		odomCurrentMeasure = log->robotPose;
		motionModel.predictOdometryModel(particleFilter.particles[0], odomPreviousMeasure, odomCurrentMeasure, worldMap, true);
		
		//set odomPreviousMeasure to odomCurrentMeasure for next iteration
		odomPreviousMeasure = odomCurrentMeasure;
		visualizeMap(worldMap, particleFilter.particles, "Particles Visualization");
    }
	return 0;
}
