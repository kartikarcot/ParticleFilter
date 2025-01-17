#include <MotionModel.hpp>


MotionModel::MotionModel(double rot1Var, double transVar, double rot2Var) : 
				processNoise(OdomModelNoise(rot1Var, transVar, rot2Var)){}

void MotionModel::predictOdometryModel(Pose2D& particlePose, Pose2D& robotPoseinOdomFramePrev, Pose2D& robotPoseinOdomFrameCurrent)
{

    double deltaY = robotPoseinOdomFrameCurrent.y-robotPoseinOdomFramePrev.y ,
            deltaX = robotPoseinOdomFrameCurrent.x-robotPoseinOdomFramePrev.x;

    double rot1 = deltaY > MINCHANGE ? atan2( deltaY,
					  deltaX) 
				- robotPoseinOdomFramePrev.theta : 0;

    double trans = sqrt(
					pow(deltaY,2.0) + 
					pow(deltaX,2.0));

    double rot2 = robotPoseinOdomFrameCurrent.theta - rot1 - robotPoseinOdomFramePrev.theta;

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
