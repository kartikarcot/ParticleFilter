#pragma once
#include<cmath>
#include<memory>
#include<Map.hpp>

#ifdef DEBUG
#define SPDLOG_ACTIVE_LEVEL SPDLOG_LEVEL_DEBUG
#endif
#include "spdlog/spdlog.h"

inline bool isFreespace(float x, float y, std::shared_ptr<Map> mp);

