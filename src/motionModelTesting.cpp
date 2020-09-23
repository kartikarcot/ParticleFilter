#include <iostream>
#include <memory>
#include <Map.hpp>
#include <LogReader.hpp>
#include <ParticleFilter.hpp>
#include <MotionModel.hpp>
#include <SensorModel.hpp>
#include <boost/optional.hpp>
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

	ParticleFilter particleFilter = ParticleFilter(1 , worldMap);
	particleFilter.particles[0] = Pose2D(4000,4000,0);
	std::vector<double> alphas = ALPHAS;
	MotionModel motionModel(ROT1_VAR, TRANS_VAR, ROT2_VAR, alphas);
	SensorModel sensorModel(
			Z_HIT,
			Z_SHORT,
			Z_MAX,
			Z_RAND,
			Z_HIT_VAR,
			Z_LAMBDA_SHORT);

	// declare some useful variables used in MCL
	bool firstTime = true;
	Pose2D odomPreviousMeasure, odomCurrentMeasure;
	Pose2D particlePreviousMeasure, particleCurrentMeasure;
	
	// create alog file for storing particle positions
	/* std::fstream fsm; */
	/* fsm.open("/Users/stark/Projects/16833_HW1_ParticleFilter/build/particlePose.txt"); */
	std::vector<Pose2D> particleHistory;
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
			motionModel.predictOdometryModel(particleFilter.particles[i], odomPreviousMeasure, odomCurrentMeasure, worldMap);
			particleHistory.push_back(particleFilter.particles[0]);
		}
		//set odomPreviousMeasure to odomCurrentMeasure for next iteration
		odomPreviousMeasure = odomCurrentMeasure;
	}
	/* fsm.close(); */
	visualizeMap(worldMap, particleHistory, "Particles History", -1);
	return 0;
}
