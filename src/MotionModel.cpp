#include <MotionModel.hpp>

#ifdef DEBUG
#define SPDLOG_ACTIVE_LEVEL SPDLOG_LEVEL_DEBUG
#endif
#include "spdlog/spdlog.h"

MotionModel::MotionModel(double rot1Var, double transVar, double rot2Var) : 
				processNoise(OdomModelNoise(rot1Var, transVar, rot2Var)){}

void MotionModel::predictOdometryModel(Pose2D& particlePose, Pose2D& odomPreviousMeasure, Pose2D& odomCurrentMeasure)
{
    double rot1 = atan2( 
					odomCurrentMeasure.y-odomPreviousMeasure.y , 
					odomCurrentMeasure.x-odomPreviousMeasure.x ) 
				- odomPreviousMeasure.theta;

    double trans = sqrt(
					pow((odomCurrentMeasure.y-odomPreviousMeasure.y),2.0) + 
					pow((odomCurrentMeasure.x-odomPreviousMeasure.x),2.0));

    double rot2 = odomCurrentMeasure.theta - rot1 - odomPreviousMeasure.theta;

    //Conceptual doubt : to add or subtract
    std::default_random_engine generator(SEED);
    double rot1Bar = rot1 - processNoise.dists[0](generator);
    double transBar = trans - processNoise.dists[0](generator);
    double rot2Bar = rot2 - processNoise.dists[0](generator);

	/* SPDLOG_DEBUG("The ut0 pose is {} {} {}", odomPreviousMeasure.x, odomPreviousMeasure.y, odomPreviousMeasure.theta); */
	/* SPDLOG_DEBUG("The ut1 pose is {} {} {}", odomCurrentMeasure.x, odomCurrentMeasure.y, odomCurrentMeasure.theta); */
	/* SPDLOG_DEBUG("The deltas are {} {} {}", rot1Bar, transBar, rot2Bar); */

    particlePose.x = particlePose.x + transBar * cos( rot1Bar + particlePose.theta);
    particlePose.y = particlePose.y + transBar * sin( rot1Bar + particlePose.theta);
    particlePose.theta += rot1Bar + rot2Bar;
}
