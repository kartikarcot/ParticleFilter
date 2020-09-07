#pragma once
#include<ParticleFilter.hpp>

class SensorModel
{
    SensorModel();

    void beam_range_finder_model(Particle& p, std::vector<int>& z_t1);
    void normalize_weights(Particle& p);
    void ray_casting(Pose2D& laser_pose);

};  