#ifndef PARTICLEFILTER_H
#define PARTICLEFILTER_H

#include <iostream>
#include <vector>
#include <random>
#include <memory>
#include <Map.hpp>
//std::default_random_engine generator;//@TODO: Fix the redefinition error on using single gen, should not need two generators in two classes
class Map;

struct Pose2D
{
    double x, y, theta;
	Pose2D(double _x, double _y, double _theta) : x(_x), y(_y), theta(_theta) {}
	Pose2D() : x(-1.0), y(-1.0), theta(-1.0) {}
};

struct Particle
{
    Pose2D pose;
    double weight;
};

class ParticleFilter
{
public:
    
    std::default_random_engine generator;
    const size_t numParticles;

    std::vector<Particle> particles;

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
};

#endif
