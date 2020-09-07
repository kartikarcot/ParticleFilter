#pragma once
#include<ParticleFilter.hpp>

class Resampler
{
    public:
        
        Resampler();
        
        void normalize(std::vector<Particle>& particles);

        void low_variance_sampling(std::vector<Particle>& particles);


        void multinomial_sampling(std::vector<Particle>& particles);
        
};