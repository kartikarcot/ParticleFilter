#pragma once
#include <iostream>
#include <vector>
#include <random>
#include <memory>
#include <Map.hpp>
//std::default_random_engine generator;//@TODO: Fix the redefinition error on using single gen, should not need two generators in two classes
struct Pose2D
{
    double x, y, theta;
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
    const size_t num_particles;

    std::vector<Particle> particles;

    std::vector<double> weights;

    // Initialise the particle filter
    ParticleFilter(const size_t num_particles, std::shared_ptr<Map> mp);

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