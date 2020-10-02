#include <MotionModel.hpp>
#include <Utils.hpp>
#include <cmath>


MotionModel::MotionModel(
						const std::vector<double> &_alphas, 
						const int& seed,
						const Pose2D& robotPoseinOdomFramePrev, 
						const Pose2D& robotPoseinOdomFrameCurrent) : 
						alphas(_alphas), 
						tgenerator(std::mt19937_64(std::random_device()())),
						rgenerator(std::mt19937_64(std::random_device()())),
						processNoise(0.001,0.01,0.001)
{

	double deltaY = robotPoseinOdomFrameCurrent.y-robotPoseinOdomFramePrev.y ,
	deltaX = robotPoseinOdomFrameCurrent.x-robotPoseinOdomFramePrev.x;

	deltaRot1 = deltaY > MINCHANGE && deltaX > MINCHANGE? 
		atan2( deltaY,deltaX) - robotPoseinOdomFramePrev.theta : 0;
	
	deltaRot1 = std::fmod(deltaRot1, 2*PI);

	deltaTrans = sqrt(
			pow(deltaY,2.0) + 
			pow(deltaX,2.0));

	deltaRot2 = robotPoseinOdomFrameCurrent.theta - deltaRot1 - robotPoseinOdomFramePrev.theta;

	deltaRot2 = std::fmod(deltaRot2, 2*PI);

	double rot1Var = alphas[0]*pow(deltaRot1,2)+alphas[1]*pow(deltaTrans,2);
	double transVar = alphas[2]*pow(deltaTrans,2)+alphas[3]*(pow(deltaRot1,2)+pow(deltaRot2,2));
	double rot2Var = alphas[0]*pow(deltaRot2,2) + alphas[1]*pow(deltaTrans,2);

	processNoise = OdomModelNoise((rot1Var),(transVar),(rot2Var));

}

bool MotionModel::predictOdometryModel(
		Pose2D& particlePose, 
		const std::shared_ptr<Map> &mp, bool ignoreObstacles)
{

    double rot1Bar = deltaRot1 - processNoise.dists[0](rgenerator);
    double transBar = deltaTrans - processNoise.dists[1](tgenerator);
    double rot2Bar = deltaRot2 - processNoise.dists[2](rgenerator);

	/* SPDLOG_DEBUG("The ut0 pose is {} {} {}", odomPreviousMeasure.x, odomPreviousMeasure.y, odomPreviousMeasure.theta); */
	/* SPDLOG_DEBUG("The ut1 pose is {} {} {}", odomCurrentMeasure.x, odomCurrentMeasure.y, odomCurrentMeasure.theta); */
	/* SPDLOG_DEBUG("The deltas are {} {} {}", rot1Bar, transBar, rot2Bar); */

    auto newX = particlePose.x + transBar * cos( rot1Bar + particlePose.theta);
    auto newY = particlePose.y + transBar * sin( rot1Bar + particlePose.theta);
    
    if( ignoreObstacles || mp->isFreespace(newX, newY) )
    {
        particlePose.x = newX;
        particlePose.y = newY;
        particlePose.theta += rot1Bar + rot2Bar;
		particlePose.theta = std::fmod(particlePose.theta, 2*PI);
        return true;
    }
    return false;
}
