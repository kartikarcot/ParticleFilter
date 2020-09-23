#include<ParticleFilter.hpp>
#include <random>
#include "config.hpp"

#define PI 3.14159


inline bool isFreespace(float x, float y, std::shared_ptr<Map> mp)
{
    return (mp->valid(x,y) && mp->at(x,y) >= 0.0 && mp->at(x,y) <= FREE_SPACE_THRESHOLD); 
}


ParticleFilter::ParticleFilter(const size_t _numParticles, std::shared_ptr<Map> mp) : 
				numParticles(_numParticles),
				weights(std::vector<double>(_numParticles, 1/double(_numParticles)))
{
    
    std::uniform_real_distribution<double> distX(mp->minX,mp->maxX), distY(mp->minY,mp->maxY), distTheta(-PI, PI);
    
    size_t numGenerated=0;
	SPDLOG_INFO("numparticles {}",numParticles);
    while(numGenerated < numParticles)
    {
        double x = distX(generator), y = distY(generator), theta = distTheta(generator);
        if(isFreespace(x,y,mp))
        {
            particles.emplace_back(Pose2D(x,y,theta));
            numGenerated++;
        }
    }
}

void ParticleFilter::predict()
{
    throw "Not implemented yet";
}

void ParticleFilter::update()
{
    throw "Not implemented yet";
}


void ParticleFilter::resample()
{
	SPDLOG_DEBUG("The weights after normalizing are");
	for (const auto &w : weights)
	{
		std::cout<<w<<" ";
	}
	std::cout<<std::endl;

	std::normal_distribution<double> x_noise(0.0,POS_VAR), y_noise(0.0,POS_VAR), theta_noise(0.0,THETA_VAR);
	std::default_random_engine generator(SEED);
    // normalize_weights(weights);
    std::discrete_distribution<int> distribution(weights.begin(), weights.end());
	std::vector<Pose2D> newParticles(numParticles);
	for (auto &newParticle : newParticles)
	{
		newParticle = particles[distribution(generator)];
		newParticle.x += x_noise(generator);
		newParticle.y += y_noise(generator);
		newParticle.theta += theta_noise(generator);
	}
	particles = std::move(newParticles);
	return;
}
/**
 * @brief 
 *  zMax range - 0.001-0.01
 *  zHit - 1
 *  zShort - 0.01-0.1
 * rand - 100-10000
 * zHitVvar - start with 1; 8000 
 * zlambda - 1 
 */
