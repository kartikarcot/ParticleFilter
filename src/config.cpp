#include "config.hpp"
#include <fstream>

std::shared_ptr<Config> Config::configPtr = nullptr;

void Config::initializeConfig(const std::string &configPathName)
{
	configPtr = std::shared_ptr<Config>(new Config(configPathName));
}

Config::Config(const std::string &configPathName)
{
	std::ifstream fsm(configPathName);
	fsm >> configJsonObject;
}

std::shared_ptr<Config> Config::getInstance()
{
	if (!configPtr)
		return nullptr;
	else
		return configPtr;
}

