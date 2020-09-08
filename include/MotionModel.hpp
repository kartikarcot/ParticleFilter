#pragma once
#include<ParticleFilter.hpp>
#include <cmath>

struct OdomModelNoise
{
    
    double rot1_var;
    double trans_var;
    double rot2_var;
    
    
    //Assume a gaussian noise for each of the motions
    std::vector<std::normal_distribution<double>> dists;

    OdomModelNoise(double rot1_noise, double trans_noise, double rot2_noise): 
    rot1_var(rot1_noise),trans_var(trans_noise),rot2_var(rot2_noise)
    {
        dists.push_back(std::normal_distribution<double>(0.0,rot1_var));
        dists.push_back(std::normal_distribution<double>(0.0,trans_var));
        dists.push_back(std::normal_distribution<double>(0.0,rot2_var));
    }
};
class MotionModel
{
    
    OdomModelNoise process_noise;
    public:
    MotionModel(double rot1_var,double trans_var,double rot2_var);

    void predictOdometryModel(Particle& p, Pose2D& u_t0, Pose2D& u_t1);


};