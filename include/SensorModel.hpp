#ifndef SENSORMODEL_H
#define SENSORMODEL_H
#include <ParticleFilter.hpp>
#include <Map.hpp>
#include <memory>
#include "config.hpp"

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
	
	SensorModel(double zHit, double zShort, double zMax, double zRand, double zHitVar, double zLambdaShort);
		
	double beamRangeFinderModel(const Pose2D &laserPoseInOdomFrame,
						const Pose2D &robotPoseInOdomFrame,
						Pose2D& particlePoseInWorldFrame, 
						std::vector<int>& realLaserData,
						const std::shared_ptr<Map> &worldMap);
		
		std::vector<double> testingData(double z_star);

	private:

		std::vector<int> rayCasting(
							const Pose2D &laserPoseInOdomFrame, 
							const Pose2D &robotPoseInOdomFrame,
							const Pose2D &particlePoseInWorldFrame, 
							const std::shared_ptr<Map> &map);

		double rayCastingstepSize = RAY_CASTING_STEP_SIZE;
		double threshold = OBSTACLE_THRESHOLD;
		double laserMaxRange = MAX_RANGE; 
		double laserMinRange = 0;
		

		//Sensor model probability parameters
		double zHit;
		double zShort;
		double zMax; // TODO: check this for the actual value
		double zRand;
		double zHitVar;
		double zLambdaShort;

		double pHit(const double &z, const double &zStar);
		double pShort(const double &z, const double &zStar);
		double pMax(const double &z, const double &zStar);
		double pRand(const double &z);
};  

#endif
