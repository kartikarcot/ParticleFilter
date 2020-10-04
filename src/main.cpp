#include <fstream>
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

	if (argc!=4)
	{
		SPDLOG_ERROR("Invalid number of arguments\n.  \
				Follow this format <path to exec> <path to map> <path to log> <path to config>");
		return 1; 
	}

    cv::VideoWriter video("pf_log.avi",CV_FOURCC('M','J','P','G'),10, cv::Size(800,800));

	Config::initializeConfig(std::string(argv[3]));
	std::shared_ptr<Config> cfg = Config::getInstance();
	bool visualizeMapFlag = cfg->get<bool>("visualizeMap");
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
									cfg->get<double>("thetavar"),
									cfg->get<int>("seed"));

    // for (int i = 0; i < 5000; i++)
    // {
    //     particleFilter.particles[i] = Pose2D(8000-4000,4140, -PI/2 - 10*PI/180);
    // }
	std::vector<double> alphas = {
							cfg->get<double>("alpha1"),
							cfg->get<double>("alpha2"),
							cfg->get<double>("alpha3"),
							cfg->get<double>("alpha4")
						};

	int seed = cfg->get<int>("seed");

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
	Profiler<std::chrono::seconds> pf("Time(s) taken to finish");
	// read logs and perform probabilistic updates
	int logNum =0;
	while((log = logReader.getLog()))
	{
		SPDLOG_DEBUG("Log number : {}", logNum++);
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

		MotionModel motionModel(alphas, seed, odomPreviousMeasure, odomCurrentMeasure);

		#pragma omp parallel for
		for (std::size_t i = 0; i<particleFilter.numParticles; i++)
		{
			// auto particlePose = particleFilter.particles[i];
			// Motion Model update
			#pragma omp critical
			{
				if(!motionModel.predictOdometryModel(
								particleFilter.particles[i], 
								worldMap, 
								false))
					particleFilter.weights[i] = -100;
			}
			// Sensor Model update
			if (log->logType == LogType::LASER)
			{
				/* if(particleFilter.weights[i]!=0) */
					particleFilter.weights[i] = sensorModel.beamRangeFinderModel(
											particleFilter.particles[i],
											log->laserdata,
											worldMap);
			}
		}

		//set odomPreviousMeasure to odomCurrentMeasure for next iteration
		odomPreviousMeasure = odomCurrentMeasure;	

		if (visualizeMapFlag)
			visualizeMapWithArrows(particleFilter, worldMap, "Particles Visualization Before Resampling", 50, video);

		if (log->logType == LogType::LASER)
			particleFilter.lowVarianceResample(worldMap);

		/* if (visualizeMapFlag) */
		/* 	visualizeMapWithArrows(particleFilter, worldMap, "Particles Visualization", 50, video); */
		
	}
	video.release();
	return 0;
}
