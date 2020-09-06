#include "Map.hpp"
#include <fstream>
#include <memory>
#include <sstream>
#include "opencv2/core/hal/interface.h"
#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

Map::Map(const std::string &fName, 
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
			autoshiftedY(autoY){};

inline bool keepGoing(std::fstream &fsm, std::string &line)
{
	return (std::getline(fsm, line) && (line.compare(0,13,"global_map[0]") != 0));
}

std::shared_ptr<Map> makeMap(const std::string &fName)
{
	std::fstream fsm(fName);
	int mapsize_x;
	int mapsize_y;
	int resolution;
	int autoshifted_x;
	int autoshifted_y;
	std::string line, word;
	std::stringstream ss;

	if (fsm.fail())
		throw "Error reading file!";

	try
	{
		while (keepGoing(fsm, line))
		{
			if (line.compare(0, 32, "robot_specifications->resolution") == 0)
			{
				ss.clear();
				ss<<line.substr(32,line.size()-32);
				ss>>resolution;
				if (ss.fail())
					throw "Invalid Resolution provided";
				std::cout<<resolution<<std::endl;
			}
			else if (line.compare(0, 35, "robot_specifications->autoshifted_y") == 0)
			{
				ss.clear();
				ss<<line.substr(35,line.size()-35);
				ss>>autoshifted_y;
				if (ss.fail())
					throw "Invalid Autoshifted Y value provided";
				std::cout<<autoshifted_y<<std::endl;
			}
			else if (line.compare(0, 35, "robot_specifications->autoshifted_x") == 0)
			{
				ss.clear();
				ss<<line.substr(35,line.size()-35);
				ss>>autoshifted_x;
				if (ss.fail())
					throw "Invalid Autoshifted X value provided";
				std::cout<<autoshifted_x<<std::endl;
			}
		}
		if (line.compare(0, 13, "global_map[0]")==0)
		{
			std::string temp;
			ss.clear();
			ss<<line;
			ss>>temp>>mapsize_x>>mapsize_y;
			if (ss.fail())
				throw "Invalid Map size provided";
			std::cout<<"Map Size :"<<mapsize_x<<" "<<mapsize_y<<std::endl;
		}
	}
	catch (const char *msg)
	{
		std::cout<<"Exception Ocurred!\n"<<msg<<std::endl;
		std::cout<<"Exiting Program"<<std::endl;
		exit(1);
	}

	// allocate a 2D array of size (mapsize_x,mapsize_y)
	std::vector<std::vector<float>> mapData(mapsize_x,std::vector<float>(mapsize_y,0));
	// we expect mapsize_x number of rows and mapsize_y number of columns in the input
	try
	{
		for (int i = 0; i < mapsize_x; i++)
		{
			for (int j = 0; j < mapsize_y; j++)
			{
				if (!(fsm>>word))
					throw "File did not have the required number of values!";
				ss.clear();
				ss<<word;
				if (!(ss>>mapData[i][j]))
					throw "The file had non-float data";
			}
		}
	}
	catch (const char *msg)
	{
		std::cout<<"Exception Ocurred!\n"<<msg<<std::endl;
		std::cout<<"Exiting Program"<<std::endl;
		exit(1);
	}

	return std::make_shared<Map>(
			fName, 
			mapData,
			mapsize_x,
			mapsize_y,
			resolution,
			autoshifted_x,
			autoshifted_y);
}

void visualizeMap(const std::shared_ptr<Map> map)
{
	cv::Mat mat;
	mat.create(map->data.size(), map->data[0].size(), CV_64FC1);

	for (int i = 0; i < map->data.size(); i++)
		for (int j = 0; j < map->data[i].size(); j++)
			mat.at<double>(i,j) = map->data[i][j];

	cv::namedWindow("Map Preview", cv::WINDOW_AUTOSIZE);
	cv::imshow("Map Preview", mat);
	cv::waitKey(0);
}
// Usage
// std::shared_ptr<Map> mp = makeMap("../data/map/wean.dat");
