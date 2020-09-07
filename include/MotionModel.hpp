#pragma once
#include<ParticleFilter.hpp>

class MotionModel
{
    public:
    MotionModel();
    
    Pose2D predict_odometry_model(Particle& p, Pose2D& x_t0, Pose2D& u_t1);


};