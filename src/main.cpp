#include <iostream>
#include <memory>
#include <Map.hpp>
#include <LogReader.hpp>
#include <ParticleFilter.hpp>
#include <MotionModel.hpp>
#include <SensorModel.hpp>
#include <boost/optional.hpp>
#include "Profiler.hpp"

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

	// initialize map, particle filter and sensor modes
	std::shared_ptr<Map> worldMap = makeMap(std::string(argv[1]));
	LogReader logReader((std::string(argv[2])));
	boost::optional<Log> log;

	const size_t numParticles = 100;
	ParticleFilter particleFilter = ParticleFilter(numParticles , worldMap);

	MotionModel motionModel(0.05, 0.05, 0.05);
	SensorModel sensorModel;

	// declare some useful variables used in MCL
	bool firstTime = true;
	Pose2D odomPreviousMeasure, odomCurrentMeasure;
	Pose2D particlePreviousMeasure, particleCurrentMeasure;
	
	// read logs and perform probabilistic updates
	while((log = logReader.getLog()))
	{
		SPDLOG_DEBUG("The log read was {} {} {}", log->robotPose.x, log->robotPose.y, log->robotPose.theta);
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
		for (auto &particlePose : particleFilter.particles)
		{
			// Motion Model update
			motionModel.predictOdometryModel(particlePose, odomPreviousMeasure, odomCurrentMeasure);

			// Sensor Model update
			if (log->logType == LogType::LASER)
			{
				sensorModel.rayCasting(
						log->laserPose,
						odomCurrentMeasure,
						particlePose,
						worldMap);
			}
		}

		//set odomPreviousMeasure to odomCurrentMeasure for next iteration
		odomPreviousMeasure = odomCurrentMeasure;
		particleFilter.resample();
		#ifdef DEBUG
			visualizeMap(worldMap, particleFilter.particles, "Particles Visualisation");
		#endif
	}
	return 0;
}
