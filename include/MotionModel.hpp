#ifndef MOTIONMODEL_H
#define MOTIONMODEL_H

#ifdef DEBUG
#define SPDLOG_ACTIVE_LEVEL SPDLOG_LEVEL_DEBUG
#endif

#include "spdlog/spdlog.h"
#include<ParticleFilter.hpp>
#include <Utils.hpp>

#define MINCHANGE 1e-8
struct OdomModelNoise
{
    
    double rot1Var;
    double transVar;
    double rot2Var;
    
    
    //Assume a gaussian noise for each of the motions
    std::vector<std::normal_distribution<double>> dists;

    OdomModelNoise(double _rot1Var, double _transVar, double _rot2Var): 
    rot1Var(_rot1Var),transVar(_transVar),rot2Var(_rot2Var)
	{
        dists.push_back(std::normal_distribution<double>(0.0,rot1Var));
        dists.push_back(std::normal_distribution<double>(0.0,transVar));
        dists.push_back(std::normal_distribution<double>(0.0,rot2Var));
    }
};


class MotionModel
{
    std::vector<double> alphas;
    OdomModelNoise processNoise;
    public:
    MotionModel(std::vector<double> alphas);
    bool predictOdometryModel(Pose2D& p, Pose2D& odomPreviousMeasure, Pose2D& odomCurrentMeasure, std::shared_ptr<Map> mp, bool ignoreObstacles);

};

#endif
