#ifndef PARTICLEFILTER_H
#define PARTICLEFILTER_H

#ifdef DEBUG
#define SPDLOG_ACTIVE_LEVEL SPDLOG_LEVEL_DEBUG
#endif
#include "spdlog/spdlog.h"

#include<config.hpp>
#include <iostream>
#include <vector>
#include <random>
#include <memory>
#include<Utils.hpp>
#include <Map.hpp>

#define SEED 0
//std::default_random_engine generator;//@TODO: Fix the redefinition error on using single gen, should not need two generators in two classes

// forward declaration to subvert circular include issue
class Map;




class ParticleFilter
{
public:
    
    std::default_random_engine generator;
    const size_t numParticles;

    std::vector<Pose2D> particles;

    std::vector<double> weights;

    // Initialise the particle filter
    ParticleFilter(const size_t _numParticles, std::shared_ptr<Map> mp);

    /* predict the state for the next time step
	 *   using the motion model.
    */
    void predict();

    /**
	 * Correction step- update the weights for each particle based on the likelihood of the 
	 *   observed measurements.
    **/
    void update();

    /*
    ** Resample particles for the next time step with probabilities proportial to their importance weights.
    */
    void resample();

	// Low variance resampling technique
	void lowVarianceResample();
	void lowVarianceResampleTest();
};

#endif
