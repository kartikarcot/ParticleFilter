#include <iostream>
#include <memory>
#include "Map.hpp"

int main(int argc, char **argv)
{
	if (argc!=2)
		return 1;
	std::shared_ptr<Map> mp = makeMap(std::string(argv[1]));
	return 0;
}
