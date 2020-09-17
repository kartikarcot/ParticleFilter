#ifndef MAP_H
#define MAP_H

#include <iostream>
#include <string.h>
#include <vector>
#include <memory>
#include <utility>
#include <ParticleFilter.hpp>

// forward declaration to subvert circular include issue
struct Pose2D;

class Map
{
	public:
		std::string fileName;
		std::vector<std::vector<float>> data;
		
		int mapSizeX;
		int mapSizeY;

		int minX, minY, maxX, maxY;
		
		int resolution;

		int autoshiftedX;
		int autoshiftedY; 
		// delete default constructor
		Map() = delete;
		// declare custom constructor
		Map(const std::string &fName, 
				const std::vector<std::vector<float>> &mapData,
				const int &xSize,
				const int &ySize,
				const int &res,
				const int &autoX,
				const int &autoY);
		// copy and move constructors are default, don't see any reason to define them.
		// destructor is default, all data is stl and will be safely deleted
};


std::shared_ptr<Map> makeMap(const std::string &fName);
void visualizeMap(
		const std::shared_ptr<Map> map, 
		const std::vector<Pose2D> &particleVector = {},
		const std::string &message = "Map Visualisation");

#endif
