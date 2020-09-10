#include <iostream>
#include <memory>
#include <Map.hpp>
#include <LogReader.hpp>
#include <ParticleFilter.hpp>
#include <MotionModel.hpp>
#include <boost/optional.hpp>

#ifdef DEBUG
#define SPDLOG_ACTIVE_LEVEL SPDLOG_LEVEL_DEBUG
#endif
#include "spdlog/spdlog.h"

int main(int argc, char **argv)
{
	spdlog::set_level(
        static_cast<spdlog::level::level_enum>(SPDLOG_ACTIVE_LEVEL));
	if (argc!=3)
		return 1; //TODO: change to std::Error
	const size_t num_particles = 200;
	std::shared_ptr<Map> mp = makeMap(std::string(argv[1]));
	
	LogReader logReader((std::string(argv[2])));
	boost::optional<Log> log;
	auto particleFilter = ParticleFilter(num_particles , mp);
	MotionModel motionModel(0.005, 0.05, 0.005);
	
	int count = 0;
	bool firstTime = false;
	while((log = logReader.getLog()))
	{
		Pose2D robotPose = log->robotPose;
		Pose2D laserPose;
		Pose2D ut0, ut1;
		Pose2D xt0, xt1;
		if (log->logType == LogType::LASER)
			laserPose = log->laserPose;
		if (firstTime)
		{
			ut0 = robotPose;
			firstTime = false;
			continue;
		}
		for (auto &particle : particleFilter.particles)
		{
			// Motion Model update
			motionModel.predictOdometryModel(particle, ut0, ut1);
			
			// Sensor Model update

			// update the particle beliefs
			ut0 = ut1;
		}
		SPDLOG_DEBUG("log read");
		sleep(1);
		visualizeMap(mp, particleFilter.particles);
	}
	SPDLOG_INFO("The number of records counted are {}",count);
	return 0;
}
