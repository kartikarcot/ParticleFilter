#include<ParticleFilter.hpp>
#include <random>

#ifdef DEBUG
#define SPDLOG_ACTIVE_LEVEL SPDLOG_LEVEL_DEBUG
#endif
#include "spdlog/spdlog.h"

#define PI 3.14159


inline bool isFreespace(float x, float y, std::shared_ptr<Map> mp)
{
    int xCell =int(x), yCell =int(y);
    return (mp->data[yCell][xCell] >= 0.0 && mp->data[yCell][xCell] <= 0.5); 
}

ParticleFilter::ParticleFilter(const size_t _numParticles, std::shared_ptr<Map> mp) : 
				numParticles(_numParticles),
				weights(std::vector<double>(_numParticles, 1/double(_numParticles)))
{
    
    std::uniform_real_distribution<double> distX(mp->minX,mp->maxX-1), distY(mp->minY,mp->maxY-1), distTheta(-PI, PI);
    
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
	std::default_random_engine generator(SEED);
	std::discrete_distribution<int> distribution(weights.begin(), weights.end());
	std::vector<Pose2D> newParticles(numParticles);
	for (auto &newParticle : newParticles)
	{
		newParticle = particles[distribution(generator)];
	}
	particles = std::move(newParticles);
	return;
}
