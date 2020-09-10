#ifndef SENSORMODEL_H
#define SENSORMODEL_H

#include <ParticleFilter.hpp>
#include <Map.hpp>
#include <memory>
class SensorModel
{
	public:
		SensorModel() {};
		double stepSize = 1;
		double threshold = 0.5;
		void beamRangeFinderModel(Particle& p, std::vector<int>& z_t1);
		void normalizeDistribution(Particle& p);
		std::vector<int> rayCasting(
							const Pose2D &laserPoseInOdomFrame, 
							const Pose2D &robotPoseInOdomFrame,
							const Pose2D &particlePoseInWorldFrame, 
							const std::shared_ptr<Map> &map);
};  

#endif
