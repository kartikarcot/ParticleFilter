#include <iostream>
#include <memory>
#include <Map.hpp>
#include <LogReader.hpp>
#include <ParticleFilter.hpp>

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
	visualizeMap(mp);
	
	LogReader lr((std::string(argv[2])));
	int count = 0;
	while(lr.getLog())
		count++;
	
	auto pf=ParticleFilter(num_particles , mp);
	
	SPDLOG_INFO("Number of logs recorded were {}",count);
	
	return 0;
}
