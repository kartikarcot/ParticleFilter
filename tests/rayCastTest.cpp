#include <iostream>
#include <memory>
#include <Map.hpp>
#include <LogReader.hpp>
#include <ParticleFilter.hpp>
#include <MotionModel.hpp>
#include <SensorModel.hpp>
#include <boost/optional.hpp>
#include <json.hpp>
#include "Profiler.hpp"
#include "config.hpp"
#include <fstream>

#ifdef DEBUG
#define SPDLOG_ACTIVE_LEVEL SPDLOG_LEVEL_DEBUG
#endif
#include "spdlog/spdlog.h"

int main(int argc, char **argv)
{
	spdlog::set_level(
        static_cast<spdlog::level::level_enum>(SPDLOG_ACTIVE_LEVEL));

	spdlog::set_pattern("%^[%l] [%s]%$ %v");
	std::fstream fsm("/Users/stark/Projects/ParticleFilter/build/laser.txt",std::ios::out);

	Config::initializeConfig(std::string(argv[3]));
	std::shared_ptr<Config> cfg = Config::getInstance();

	if (argc!=4)
	{
		SPDLOG_ERROR("Invalid number of arguments\n.  \
				Follow this format <path to exec> <path to map> <path to log>");
		return 1; 
	}

	// initialize map, particle filter and sensor modes
	std::shared_ptr<Map> worldMap = makeMap(
									std::string(argv[1]),
									cfg->get<double>("freespacethreshold"),
									cfg->get<double>("obstaclethreshold"));
	LogReader logReader((std::string(argv[2])));
	boost::optional<Log> log;

	ParticleFilter particleFilter = ParticleFilter(
									cfg->get<int>("num_particles") , 
									worldMap,
									cfg->get<double>("posvar"),
									cfg->get<double>("thetavar"));

	particleFilter.particles[0] = Pose2D(8000-3950,4560, -PI/2);
	/* particleFilter.particles[0] = Pose2D(8000-7100,4400, -PI/2); */
	SensorModel sensorModel(
			cfg->get<double>("zHit"),
			cfg->get<double>("zShort"),
			cfg->get<double>("zMax"),
			cfg->get<double>("zRand"),
			cfg->get<double>("zHitVar"),
			cfg->get<double>("zLambdaShort"),
			cfg->get<double>("raycastingstepsize"),
			cfg->get<double>("maxrange"),
			cfg->get<int>("rayskipfactor"),
			cfg->get<bool>("visualizeRays"));

	// declare some useful variables used in MCL
	bool firstTime = true;
	Pose2D odomPreviousMeasure, odomCurrentMeasure;
	Pose2D particlePreviousMeasure, particleCurrentMeasure;
	
	// read logs and perform probabilistic updates
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

		for (std::size_t i = 0; i<particleFilter.particles.size(); i++)
		{
			// auto particlePose = particleFilter.particles[i];
			// Motion Model update

			// Sensor Model update
			if (log->logType == LogType::LASER)
			{
				auto values = sensorModel.rayCasting(
											log->laserPose,
											odomCurrentMeasure,
											particleFilter.particles[i],
											worldMap);
				for (int i = 0; i < values.size(); i++)
				{
					fsm<<values[i]<<" "<<log->laserdata[i]<<std::endl;
				}
				fsm.close();
				return 1;
			}
		}

		//set odomPreviousMeasure to odomCurrentMeasure for next iteration
		odomPreviousMeasure = odomCurrentMeasure;
	}
	return 0;
}
