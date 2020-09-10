#ifndef PROFILER_H
#define PROFILER_H

#include <chrono>
#include <string>

#ifdef DEBUG
#define SPDLOG_ACTIVE_LEVEL SPDLOG_LEVEL_DEBUG
#endif
#include "spdlog/spdlog.h"

template<typename T>
class Profiler
{
	public:
		std::chrono::time_point<std::chrono::high_resolution_clock> begin;
		std::string msg;
		Profiler(const std::string &_msg): msg(_msg)
		{
			begin = std::chrono::high_resolution_clock::now();
		}

		~Profiler()
		{

			std::chrono::time_point<std::chrono::high_resolution_clock>now = std::chrono::high_resolution_clock::now();
			auto timeTaken = std::chrono::duration_cast<T>(now - begin);
			SPDLOG_INFO("{} : {}",msg, timeTaken.count());
		}
};

#endif
