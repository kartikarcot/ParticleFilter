#include <fstream>
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
#include <json.hpp>

#ifdef DEBUG
#define SPDLOG_ACTIVE_LEVEL SPDLOG_LEVEL_DEBUG
#endif
#include "spdlog/spdlog.h"

int main(int argc, char **argv)
{
	spdlog::set_level(
        static_cast<spdlog::level::level_enum>(SPDLOG_ACTIVE_LEVEL));
	
	spdlog::set_pattern("%^[%l] [%s]%$ %v");

	if (argc<=4)
	{
		SPDLOG_ERROR("Invalid number of arguments\n.  \
				Follow this format <path to exec> <path to map> <path to log>");
		return 1;
	}

	Config::initializeConfig(std::string(argv[3]));
	std::shared_ptr<Config> cfg = Config::getInstance();

	// initialize map, particle filter and sensor modes
	std::shared_ptr<Map> worldMap = makeMap(
									std::string(argv[1]),
									cfg->get<double>("freespacethreshold"),
									cfg->get<double>("obstaclethreshold"));
	LogReader logReader((std::string(argv[2])));
	boost::optional<Log> log;

    ParticleFilter particleFilter = ParticleFilter(
									1, 
									worldMap,
									cfg->get<double>("posvar"),
									cfg->get<double>("thetavar"));

	std::vector<double> alphas = {
							cfg->get<double>("alpha1"),
							cfg->get<double>("alpha2"),
							cfg->get<double>("alpha3"),
							cfg->get<double>("alpha4")
						};

	MotionModel motionModel(alphas);
	double posX = stod(std::string(argv[4])); //3400
	double posY = stod(std::string(argv[5])); //4250
	double theta = stod(std::string(argv[6])); //-300/180
	particleFilter.particles[0] = Pose2D(posX,posY,TO_RADIANS(theta));
	
	bool firstTime = true;
	Pose2D odomPreviousMeasure, odomCurrentMeasure;
	Pose2D particlePreviousMeasure, particleCurrentMeasure;
	std::vector<Pose2D> trajectory;
	
	int logNum =0;
	while((log = logReader.getLog()))
	{
		SPDLOG_DEBUG("Log number : ", logNum++);
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
		trajectory.push_back(particleFilter.particles[0]);
		
		//set odomPreviousMeasure to odomCurrentMeasure for next iteration
		odomPreviousMeasure = odomCurrentMeasure;	
    }
	visualizeMap(worldMap, trajectory, "Particles Visualization", -1);
	return 0;
}
