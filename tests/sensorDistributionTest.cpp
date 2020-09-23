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
#include <jsoncpp/json/json.h>
#include <jsoncpp/json/writer.h>
#ifdef DEBUG
#define SPDLOG_ACTIVE_LEVEL SPDLOG_LEVEL_DEBUG
#endif
#include "spdlog/spdlog.h"

int main(int argc, char **argv)
{
	SensorModel sensorModel(
			Z_HIT,
			Z_SHORT,
			Z_MAX,
			Z_RAND,
			Z_HIT_VAR,
			Z_LAMBDA_SHORT);
	double z_star = 1000;
	auto z_values = sensorModel.testingData(z_star);
	std::fstream fsm;
	fsm.open("/Users/stark/Projects/16833_HW1_ParticleFilter/build/z_vals.txt");
	for (const auto &val : z_values)
		fsm<<val<<std::endl;
	fsm.close();
}
