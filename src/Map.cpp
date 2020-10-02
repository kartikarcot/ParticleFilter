#include "Map.hpp"
#include <fstream>
#include <memory>
#include <sstream>
#include "opencv2/core/hal/interface.h"
#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include "ParticleFilter.hpp"


// @TODO: 
Map::Map(
		const std::string &fName, 
		const std::vector<std::vector<float>> &mapData,
		const int &xSize,
		const int &ySize,
		const int &res,
		const int &autoX,
		const int &autoY,
		const double &freespaceThresh,
		const double &obstacleThresh) : 
		fileName(fName),
		data(mapData),
		mapSizeX(xSize),
		mapSizeY(ySize),
		resolution(res),
		autoshiftedX(autoX),
		autoshiftedY(autoY),
		freespaceThreshold(freespaceThresh),
		obstacleThreshold(obstacleThresh)
	{
		SPDLOG_INFO("The size of the data vector is {} {}", mapData.size(), mapData[0].size());
		minX=0;
		minY=0;
		maxX=xSize;
		maxY=ySize;
	}

float Map::at(float x, float y)
{
	int indexX = std::round(x/resolution);
	int indexY = std::round(y/resolution);

	return data[indexX][indexY];
}

bool Map::valid(float x, float y)
{
	int indexX = std::round(x/resolution);
	int indexY = std::round(y/resolution);
	return ((indexY >= 0) &&
		(indexY < data[0].size()) && 
		(indexX >= 0) && 
		(indexX < data.size()));
}

inline bool keepGoing(std::fstream &fsm, std::string &line)
{
return (std::getline(fsm, line) && (line.compare(0,13,"global_map[0]") != 0));
}

bool Map::isFreespace(const double &x, const double &y)
{
	if (valid(x,y))
	{
		const float &val = at(x,y);
		return (val >= 0 && val < freespaceThreshold);
	}
	return false;
}

std::shared_ptr<Map> makeMap(const std::string &fName,
						const double &freespaceThreshold,
						const double &obstacleThreshold)
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
				SPDLOG_INFO("RESOLUTION: {}", resolution);
			}
			else if (line.compare(0, 35, "robot_specifications->autoshifted_y") == 0)
			{
				ss.clear();
				ss<<line.substr(35,line.size()-35);
				ss>>autoshifted_y;
				if (ss.fail())
					throw "Invalid Autoshifted Y value provided";
				SPDLOG_INFO("AUTOSHIFTED Y: {}", autoshifted_y);
			}
			else if (line.compare(0, 35, "robot_specifications->autoshifted_x") == 0)
			{
				ss.clear();
				ss<<line.substr(35,line.size()-35);
				ss>>autoshifted_x;
				if (ss.fail())
					throw "Invalid Autoshifted X value provided";
				SPDLOG_INFO("AUTOSHIFTED X: {}", autoshifted_x);
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
			SPDLOG_INFO("MAPSIZE {} X {}", mapsize_x, mapsize_y);
		}
	}
	catch (const char *msg)
	{
		SPDLOG_ERROR("{}",msg);
		exit(1);
	}

	// allocate a 2D array of size (mapsize_y,mapsize_x)
	std::vector<std::vector<float>> mapData(mapsize_x,std::vector<float>(mapsize_y,0));
	// we expect mapsize_x number of rows and mapsize_y number of columns in the input
	float val;
	try
	{
		int count = 0;
		for (int i = 0; i < mapsize_x; i++)
		{
			for (int j = 0; j < mapsize_y; j++)
			{
				if (!(fsm>>word))
					throw "File did not have the required number of values!";

				ss.clear();
				ss<<word;

				if (!(ss>>val))
					throw "The file had non-float data";
				else
				{
					if (val < 0)
						mapData[i][j] = -1;
					else
						mapData[i][j] = 1-val;
				}

				if ((count++ % 50000) == 0)
				{
					float percentage = 100*(count)/(float)(mapsize_x*mapsize_y);
					SPDLOG_INFO("{:3.2f} reading completed",percentage);
				}
			}
		}
	}
	catch (const char *msg)
	{
		SPDLOG_ERROR("{}",msg);
		exit(1);
	}

	return std::make_shared<Map>(
			fName, 
			mapData,
			mapsize_x*resolution,
			mapsize_y*resolution,
			resolution,
			autoshifted_x,
			autoshifted_y,
			freespaceThreshold,
			obstacleThreshold);
}

void visualizeMap(
		const std::shared_ptr<Map> map, 
		const std::vector<Pose2D> &particleVector,
		const std::string &message,
		int timeout)
{
	cv::Mat mat;
	mat.create(map->data.size(), map->data[0].size(), CV_64FC3);

	for (int i = 0; i < map->data.size(); i++)
	{
		for (int j = 0; j < map->data[i].size(); j++)
		{
			double grayCode = map->data[i][j];
			// color don't know cells as occupied
			if (grayCode == -1) grayCode = 1.0;
			mat.at<cv::Vec3d>(i,j) = cv::Vec3d(grayCode, grayCode, grayCode);
		}
	}

	
	for (const auto &particlePose : particleVector)
	{
		cv::circle(mat, 
				cv::Point2d(particlePose.y/map->resolution, particlePose.x/map->resolution), 
				1, cv::Scalar(0,0,255), -1); 
	}

	
	cv::circle(mat, 
			cv::Point2d(
				particleVector[particleVector.size()-1].y/map->resolution, 
				particleVector[particleVector.size()-1].x/map->resolution), 
			2,
			cv::Scalar(0,255,0),
			-1); 

	cv::namedWindow(message, cv::WINDOW_AUTOSIZE);
	cv::imshow(message, mat);
	cv::waitKey(timeout);
}

void visualizeMapWithArrows(const ParticleFilter& pf,
							const std::shared_ptr<Map> &map,
							const std::string &message,
							const int &timeout,
							cv::VideoWriter video)
{
	cv::Mat mat1Channel, mat;
	mat1Channel.create(map->data.size(), map->data[0].size(), CV_8UC1);
	mat.create(map->data.size(), map->data[0].size(), CV_8UC3);

	for (int i = 0; i < map->data.size(); i++)
	{
		for (int j = 0; j < map->data[i].size(); j++)
		{
			double grayCode = map->data[i][j];
			// color don't know cells as occupied
			if (grayCode == -1) grayCode = 1.0;
			mat1Channel.at<uint8_t>(i,j) = (uint8_t)(255*grayCode);
		}
	}
    cv::cvtColor(mat1Channel, mat, cv::COLOR_GRAY2BGR);

	std::vector<std::pair<Pose2D, double>> particleWeightPair(pf.numParticles);
	for (int i = 0; i < pf.numParticles; i++)
	{
		particleWeightPair[i] = std::make_pair(pf.particles[i], pf.weights[i]);
	}
	std::sort(particleWeightPair.begin(), particleWeightPair.end(),
				[] (const auto &a, const auto &b) {return a.second < b.second;});

	int i = 0;
	for (const auto &particleWeightPair : particleWeightPair)
	{
		auto &particlePose = particleWeightPair.first;
		if ((i == 0) || (i == pf.numParticles-1))
		{
			double arrowLength = 100;
			const Pose2D &bestWeightedParticlePose = particlePose;
			Pose2D bestWeightedParticlePoseExtend = bestWeightedParticlePose;
			bestWeightedParticlePoseExtend.x += arrowLength*cos(particlePose.theta); 
			bestWeightedParticlePoseExtend.y += arrowLength*sin(particlePose.theta); 
			cv::Point2d start(bestWeightedParticlePose.y/map->resolution, bestWeightedParticlePose.x/map->resolution);
			cv::Point2d end(bestWeightedParticlePoseExtend.y/map->resolution, bestWeightedParticlePoseExtend.x/map->resolution);
			cv::arrowedLine(mat, start, end, CV_RGB(255, 255, 0), 1, 8, 0, 0.2);
		}
		if (i < 100)
		{
			cv::circle(mat, 
					cv::Point2d(particlePose.y/map->resolution, particlePose.x/map->resolution), 
					1, cv::Scalar(0,0,255), -1); 
		}
		else if (i > pf.numParticles - 100)
		{
			cv::circle(mat, 
					cv::Point2d(particlePose.y/map->resolution, particlePose.x/map->resolution), 
					1, cv::Scalar(0,255,0), -1); 
		}
		else
		{
			cv::circle(mat, 
					cv::Point2d(particlePose.y/map->resolution, particlePose.x/map->resolution), 
					1, cv::Scalar(255,0,0), -1); 
		}
        i++;
	}

	// draw an arrow for the best weighted particle
	cv::namedWindow(message, cv::WINDOW_AUTOSIZE);
    video.write(mat);
	cv::imshow(message, mat);
	cv::waitKey(timeout);
}
