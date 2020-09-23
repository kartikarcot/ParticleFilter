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

int main(int argc, char **argv)
{
	spdlog::set_level(
        static_cast<spdlog::level::level_enum>(SPDLOG_ACTIVE_LEVEL));

	spdlog::set_pattern("%^[%l] [%s]%$ %v");
	std::shared_ptr<Map> worldMap = makeMap(std::string(argv[1]));
	ParticleFilter particleFilter = ParticleFilter(4 , worldMap);
	std::vector<double> weights = {10, 40, 60, 30};
	particleFilter.weights = weights;
	// read logs and perform probabilistic updates
	particleFilter.lowVarianceResampleTest();
	return 0;
}
