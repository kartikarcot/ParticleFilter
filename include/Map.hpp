#include <iostream>
#include <string.h>
#include <vector>
#include <memory>
#include <utility>

class Map
{
	public:
		const std::string fileName;
		const std::vector<std::vector<float>> data;
		const int mapSizeX;
		const int mapSizeY;
		const int resolution;
		const int autoshiftedX;
		const int autoshiftedY; 
		// delete default constructor
		Map() = delete;
		// delete copy assignment operation explicitly since all data is const
		Map& operator= (const Map &rhs) = delete;
		// declare custom constructor
		Map(const std::string &fName, 
				const std::vector<std::vector<float>> &mapData,
				const int &xSize,
				const int &ySize,
				const int &res,
				const int &autoX,
				const int &autoY) : 
			fileName(fName),
			data(mapData),
			mapSizeX(xSize),
			mapSizeY(ySize),
			resolution(res),
			autoshiftedX(autoX),
			autoshiftedY(autoY)
		{};
		// copy and move constructors are default, don't see any reason to define them.
		// destructor is default, all data is stl and will be safely deleted
};


std::shared_ptr<Map> makeMap(const std::string &fName);
