#pragma once
#include<cmath>
#include<memory>
#include<Map.hpp>
#include<config.hpp>
#ifdef DEBUG
#define SPDLOG_ACTIVE_LEVEL SPDLOG_LEVEL_DEBUG
#endif
#include "spdlog/spdlog.h"
#define PI 3.14159
#define SQRT1_2 0.7071
#define TO_RADIANS(x) (PI*x/180.0)
#define SQRT_2PI 2.5066



void normalizeAndShiftWeights(std::vector<double>& weights);


