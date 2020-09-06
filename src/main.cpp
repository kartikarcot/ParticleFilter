#include <iostream>
#include <memory>
#include "Map.hpp"
#include "LogReader.hpp"

int main(int argc, char **argv)
{
	if (argc!=3)
		return 1;
	std::shared_ptr<Map> mp = makeMap(std::string(argv[1]));
	visualizeMap(mp);
	LogReader lr((std::string(argv[2])));
	int count = 0;
	while(lr.getLog())
		count++;
	std::cout<<"Number of logs recorded were "<<count<<std::endl;
	return 0;
}
