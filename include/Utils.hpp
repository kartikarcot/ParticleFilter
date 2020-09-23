#pragma once
#include<cmath>
#include<memory>
#include<Map.hpp>
#include <jsoncpp/json/json.h>
#include <jsoncpp/json/writer.h>

#ifdef DEBUG
#define SPDLOG_ACTIVE_LEVEL SPDLOG_LEVEL_DEBUG
#endif
#include "spdlog/spdlog.h"

void normalize_weights(std::vector<double>& weights);


