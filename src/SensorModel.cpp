#include <opencv2/opencv.hpp>
#include "SensorModel.hpp"
#include "Profiler.hpp"
#include "Map.hpp"

inline bool keepCasting(
		const double &x,
		const double &y,
		const std::shared_ptr<Map> &worldMap)
{
	if (worldMap->valid(x,y))
	{
		const float &val = worldMap->at(x,y);
		return (val > 0 && val < worldMap->obstacleThreshold);
	}
	return false;
}

SensorModel::SensorModel(double _zHit, 
		double _zShort, 
		double _zMax, 
		double _zRand, 
		double _zHitVar, 
		double _zLambdaShort,
		double _rayCastingStepSize,
		double _laserMaxRange,
		int _raySkipFactor,
		bool _visualizeRays): 
	zHit(_zHit), 
	zShort(_zShort), 
	zMax(_zMax), 
	zRand(_zRand), 
	zHitVar(_zHitVar), 
	zLambdaShort(_zLambdaShort),
	rayCastingstepSize(_rayCastingStepSize),
	laserMaxRange(_laserMaxRange),
	rayskipfactor(_raySkipFactor),
	visualizeRays(_visualizeRays)
{}

std::vector<int> SensorModel::rayCasting(
					const Pose2D &laserPoseInOdomFrame,
					const Pose2D &robotPoseInOdomFrame,
					const Pose2D &particlePoseInWorldFrame,
					const std::shared_ptr<Map> &worldMap)
{
	// get the laser pose in world frame for the particle belief
	double deltaX = laserPoseInOdomFrame.x - robotPoseInOdomFrame.x;
	double deltaY = laserPoseInOdomFrame.y - robotPoseInOdomFrame.y;
	double deltaTheta = laserPoseInOdomFrame.theta - robotPoseInOdomFrame.theta;
	Pose2D laserPoseInWorldFrame(
			particlePoseInWorldFrame.x + deltaX,
			particlePoseInWorldFrame.y +deltaY,
			particlePoseInWorldFrame.theta + deltaTheta);

	// perform ray casting from the laserPoseInWorldFrame
	std::vector<int> simulatedRayCast(180/rayskipfactor,laserMaxRange);
	std::vector<Pose2D> rayPoints;
	// iterate from 0 to 180 degrees

	// SPDLOG_DEBUG("ORIGIN ({},{}): {}", 
	//		std::round(laserPoseInWorldFrame.x), 
	//		std::round(laserPoseInWorldFrame.y), 
	//		worldMap->data[std::round(laserPoseInWorldFrame.y)][std::round(laserPoseInWorldFrame.x)]);
	for (int sweepAngle = 0, index = 0; sweepAngle < 180; sweepAngle+=rayskipfactor)
	{
		double slopeAngle = PI/2 - TO_RADIANS(sweepAngle) + laserPoseInWorldFrame.theta;
		double xNew = laserPoseInWorldFrame.x;
		double yNew = laserPoseInWorldFrame.y;
		int count = 0;
		while (count*rayCastingstepSize < laserMaxRange && 
				keepCasting(xNew, yNew, worldMap))
		{
			xNew = laserPoseInWorldFrame.x + count*rayCastingstepSize*cos(slopeAngle);
			yNew = laserPoseInWorldFrame.y + count*rayCastingstepSize*sin(slopeAngle);

			if (visualizeRays)
				rayPoints.push_back(Pose2D(xNew, yNew, 0));

			count++;
		}
		simulatedRayCast[index++] = (int)std::min(laserMaxRange,std::max(0.0,std::round(rayCastingstepSize*(count-1))));
	}

	if (visualizeRays)
	{
		rayPoints.push_back(laserPoseInWorldFrame);
		visualizeMap(worldMap, rayPoints, "Raycast visualization", 10);
	}
	return simulatedRayCast;
}


double SensorModel::beamRangeFinderModel(const Pose2D &laserPoseInOdomFrame,
							const Pose2D &robotPoseInOdomFrame,
							Pose2D& particlePoseInWorldFrame, 
							std::vector<int>& realLaserData,
							const std::shared_ptr<Map> &worldMap)
{
	std::vector<int> simulatedLaserData = rayCasting(laserPoseInOdomFrame,
											robotPoseInOdomFrame,
											particlePoseInWorldFrame,
											worldMap);
	
	double logProb = 0;
	for (int i = 0, j = 0 ; i< realLaserData.size() ; i+=rayskipfactor)
	{
		double realMeas = (double)realLaserData[i], simMeas = (double)simulatedLaserData[j++];
		logProb += log (zHit * pHit(realMeas,simMeas)
								+ zShort * pShort(realMeas,simMeas) 
								+ zMax * pMax(realMeas,simMeas) 
								+ zRand * pRand(realMeas));
								
	}
	/* SPDLOG_DEBUG("The logProb value is {}", logProb); */
	return exp(logProb);
	
}

std::vector<double> SensorModel::testingData(double z_star)
{
	std::vector<double> z_values(laserMaxRange);
	for (size_t i = 0; i < size_t(laserMaxRange*1.1); i++)
	{
		
		z_values[i] =  zHit * pHit((double)i,z_star)
								+ zShort * pShort((double)i,z_star)
								+ zMax * pMax((double)i,z_star)
								+ zRand * pRand((double)i);
	}
	return z_values;
}

inline double SensorModel::pHit(const double &z, const double &zStar)
{
	
	return z >= laserMinRange && z < laserMaxRange ?  1/(zHitVar*SQRT_2PI) * 
											exp( -0.5 * pow((z - zStar)/zHitVar , 2)) : 0;	
}

inline double SensorModel::pShort(const double &z, const double &zStar)
{
	return z >= laserMinRange && z < zStar ? zLambdaShort * exp(-1 * zLambdaShort * z)  : 0;
		
}

inline double SensorModel::pMax(const double &z, const double &zStar)
{
	return z >= laserMaxRange ? 1 : 0 ; //Doubt : greater than equal o or just equal to
}

inline double SensorModel::pRand(const double &z)
{
	return z >= laserMinRange && z < laserMaxRange ? 1/laserMaxRange : 0 ;
}
