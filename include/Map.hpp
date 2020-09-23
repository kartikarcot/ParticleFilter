#ifndef MAP_H
#define MAP_H

#include <iostream>
#include <string.h>
#include <vector>
#include <memory>
#include <opencv2/core/hal/interface.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <utility>
#ifdef DEBUG
#define SPDLOG_ACTIVE_LEVEL SPDLOG_LEVEL_DEBUG
#endif
#include "spdlog/spdlog.h"

// forward declaration to subvert circular include issue
struct Pose2D
{
    double x, y, theta;
	Pose2D(double _x, double _y, double _theta) : x(_x), y(_y), theta(_theta) {}
	Pose2D() : x(-1.0), y(-1.0), theta(-1.0) {}
};

class Map
{
	public:
		std::string fileName;
		std::vector<std::vector<float>> data;

		double freespaceThresh;
		double obstacleThresh;

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
		float at(float x, float y);
		bool valid(float x, float y);
};


std::shared_ptr<Map> makeMap(const std::string &fName);
void visualizeMap(
		const std::shared_ptr<Map> map, 
		const std::vector<Pose2D> &particleVector = {},
		const std::string &message = "Map Visualization",
		int timeout = 500);

#endif
