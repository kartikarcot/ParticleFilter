#include <opencv2/opencv.hpp>
#include "SensorModel.hpp"
#include "Profiler.hpp"
#include "Map.hpp"



inline bool keepCasting(
		const double &x,
		const double &y,
		const double &threshold,
		const std::shared_ptr<Map> &worldMap)
{
	return (worldMap->valid(x,y) && worldMap->at(x,y) >= 0 && worldMap->at(x,y) < threshold);
}

SensorModel::SensorModel(double _zHit, double _zShort, double _zMax, double _zRand)
: zHit(_zHit), zShort(_zShort), zMax(_zMax), zRand(_zRand)
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
	std::vector<int> simulatedRayCast(180,INT_MAX);
	std::vector<Pose2D> rayPoints;
	// iterate from 0 to 180 degrees

	// SPDLOG_DEBUG("ORIGIN ({},{}): {}", 
	//		std::round(laserPoseInWorldFrame.x), 
	//		std::round(laserPoseInWorldFrame.y), 
	//		worldMap->data[std::round(laserPoseInWorldFrame.y)][std::round(laserPoseInWorldFrame.x)]);

	for (int sweepAngle = 1; sweepAngle <=180; sweepAngle++)
	{
		double slopeAngle = TO_RADIANS(sweepAngle) + laserPoseInWorldFrame.theta;
		double xNew = laserPoseInWorldFrame.x;
		double yNew = laserPoseInWorldFrame.y;
		int count = 0;
		while (keepCasting(xNew, yNew, threshold, worldMap))
		{
			xNew = laserPoseInWorldFrame.x + count*rayCastingstepSize*cos(slopeAngle);
			yNew = laserPoseInWorldFrame.y + count*rayCastingstepSize*sin(slopeAngle);
			rayPoints.push_back(Pose2D(xNew, yNew, 0));
			count++;
		}
		simulatedRayCast[sweepAngle-1] = (int)std::min(laserMaxRange,std::round(rayCastingstepSize*(count-1)));
	}
	rayPoints.push_back(laserPoseInWorldFrame);
	#ifdef DEBUG
		// visualizeMap(worldMap, rayPoints, "Raycast visualization");
	#endif
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
	for (size_t i=0 ; i< realLaserData.size() ; i++)
	{
		size_t realMeas = realLaserData[i], simMeas = simulatedLaserData[i];
		
		logProb += log (zHit * pHit(realMeas,simMeas) 
								+ zShort * pShort(realMeas,simMeas) 
								+ zMax * pMax(realMeas,simMeas) 
								+ zRand * pRand(realMeas));
								
	}
	SPDLOG_DEBUG("The logProb value is {}", logProb);
	return exp(logProb);
	
}




inline double SensorModel::pHit(size_t &z, size_t &zStar)
{
	
	double normalizer =  2.0 / (erf( SQRT1_2 * (laserMaxRange-zStar)/zHitVar ) - 
								erf( SQRT1_2 * (laserMinRange-zStar)/zHitVar) );
	
	return z >= laserMinRange && z < zMax ? normalizer * 1/(zHitVar*SQRT_2PI) * 
											exp( 1/2 * pow(- (z - zStar)/zHitVar , 2)) : 0;	
}

inline double SensorModel::pShort(size_t &z, size_t &zStar)
{
	double normalizer = 1/(1-exp(-1 * zLambdaShort * zStar));
	return z >= laserMinRange && z < zStar ? normalizer * zLambdaShort * exp(-1 * zLambdaShort * z)  : 0;
		
}

inline double SensorModel::pMax(size_t &z, size_t &zStar)
{
	return z >= zMax ? 1 : 0 ; //Doubt : greater than equal o or just equal to
}

inline double SensorModel::pRand(size_t &z)
{
	return z >= laserMinRange && z < zMax ? 1/zMax : 0 ;
}
