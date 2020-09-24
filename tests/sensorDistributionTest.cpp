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

#ifdef DEBUG
#define SPDLOG_ACTIVE_LEVEL SPDLOG_LEVEL_DEBUG
#endif
#include "spdlog/spdlog.h"
#include <json.hpp>

int main(int argc, char **argv)
{
	
	Config::initializeConfig(std::string(argv[3]));
	std::shared_ptr<Config> cfg = Config::getInstance();
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

	double z_star = stod(std::string(argv[4]));
	auto z_values = sensorModel.testingData(z_star);
	std::fstream fsm;
	fsm.open("z_vals.txt",std::ios::out);
	for (const auto &val : z_values)
		fsm<<val<<std::endl;
	fsm.close();
}
