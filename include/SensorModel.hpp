#ifndef SENSORMODEL_H
#define SENSORMODEL_H
#include <ParticleFilter.hpp>
#include <Map.hpp>
#include <memory>

#ifdef DEBUG
#define SPDLOG_ACTIVE_LEVEL SPDLOG_LEVEL_DEBUG
#endif
#include "spdlog/spdlog.h"

#define PI 3.14159
#define SQRT1_2 0.7071
#define TO_RADIANS(x) (PI*x/180.0)
#define SQRT_2PI 2.5066

class SensorModel
{

	public:
	
	SensorModel(double zHit, double zShort, double zMax, double zRand);
		
	double beamRangeFinderModel(const Pose2D &laserPoseInOdomFrame,
						const Pose2D &robotPoseInOdomFrame,
						Pose2D& particlePoseInWorldFrame, 
						std::vector<int>& realLaserData,
						const std::shared_ptr<Map> &worldMap);
		
		

	private:

		std::vector<int> rayCasting(
							const Pose2D &laserPoseInOdomFrame, 
							const Pose2D &robotPoseInOdomFrame,
							const Pose2D &particlePoseInWorldFrame, 
							const std::shared_ptr<Map> &map);

		double rayCastingstepSize = 0.5;
		double threshold = 0.5;
		double laserMaxRange = 2000; 
		double laserMinRange = 0.0;
		

		//Sensor model probability parameters
		double zHit;
		double zShort;
		double zMax; // TODO: check this for the actual value
		double zRand;
		double zHitVar;
		double zLambdaShort;

		double pHit(size_t &z, size_t &zStar);
		double pShort(size_t &z, size_t &zStar);
		double pMax(size_t &z, size_t &zStar);
		double pRand(size_t &z);
};  

#endif
