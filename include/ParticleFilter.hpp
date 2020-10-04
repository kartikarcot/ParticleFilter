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
#include <Utils.hpp>
#include <Map.hpp>

#define SEED 1
//std::default_random_engine generator;//@TODO: Fix the redefinition error on using single gen, should not need two generators in two classes

// forward declaration to subvert circular include issue
struct Pose2D;
class Map;




class ParticleFilter
{
public:
    
    size_t numParticles;
    double movingAverageOfBelief = 0;
    double windowSize = 5;
    std::vector<Pose2D> particles;

	int particlesChangedSince = 0;
	int noiseAddedSince = 10;
	int initializedSince = 0;
	double increaseParticlesThreshold = -26.5;
	double addNoiseThreshold = -28.5;
	double kidnappedThreshold = 9;

    std::vector<double> weights;
	double posVar, thetaVar;
    // Initialise the particle filter
    ParticleFilter(const size_t _numParticles,
			const std::shared_ptr<Map> &mp,
			const double &posVar,
			const double &thetaVar,
			const int &seed = 0);

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
	void lowVarianceResample(const std::shared_ptr<Map> &mp, const int &seed = 0);
	void lowVarianceResampleTest(const std::shared_ptr<Map> &mp);
	bool updateMovingAverage();
	void initialise(const int &seed, const std::shared_ptr<Map> &mp);
};

#endif
