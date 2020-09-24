#include "json.hpp"
#include "config.hpp"
#include <fstream>

int main(int argc, char **argv)
{
	std::cout<<std::string(argv[1])<<std::endl;
	Config::initializeConfig(std::string(argv[1]));
	std::shared_ptr<Config> cfgPtr = Config::getInstance();
	std::cout<<cfgPtr->get<double>("zLambdaShort")<<std::endl;
}
