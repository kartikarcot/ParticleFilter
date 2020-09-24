#ifndef CONFIG_H
#define CONFIG_H

#include <iostream>
#include <memory>
#include "json.hpp"

class Config
{
	public:
		// delete copy constructor and assignment operators
		Config(Config &other) = delete;
		void operator=(const Config &) = delete;
		static void initializeConfig(const std::string &configPathName);
		static std::shared_ptr<Config> getInstance();
		template <typename T>
		T get(const std::string &configName);
		// public data members

	private:
		Config(const std::string &configPathName);
		static std::shared_ptr<Config> configPtr;
		nlohmann::json configJsonObject;
};

template <typename T>
T Config::get(const std::string &configName)
{
	return configJsonObject.at(configName).get<T>();
}

#endif