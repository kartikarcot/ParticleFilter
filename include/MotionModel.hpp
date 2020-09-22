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
    
    double rotVar;
    double transVar;
    
    
    //Assume a gaussian noise for each of the motions
    std::vector<std::normal_distribution<double>> dists;

    OdomModelNoise(double _rotVar, double _transVar): 
    rotVar(_rotVar),transVar(_transVar)
	{
        dists.push_back(std::normal_distribution<double>(0.0,rotVar));
        dists.push_back(std::normal_distribution<double>(0.0,transVar));
        dists.push_back(std::normal_distribution<double>(0.0,rotVar));
    }
};


class MotionModel
{

    OdomModelNoise processNoise;
    public:
    MotionModel(double _rotVar,double _transVar);
    void predictOdometryModel(Pose2D& p, Pose2D& odomPreviousMeasure, Pose2D& odomCurrentMeasure, std::shared_ptr<Map> mp);

};

#endif
