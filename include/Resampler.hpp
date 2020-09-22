#ifndef RESAMPLER_H
#define RESAMPLER_H

#include<ParticleFilter.hpp>

class Resampler
{
    public:
        
        Resampler();
        
        void normalize(std::vector<double>& weights);

        void low_variance_sampling(std::vector<double>& weights);

        void multinomial_sampling(std::vector<double>& weights);
        
};

#endif
