#include<ParticleFilter.hpp>

#ifdef DEBUG
#define SPDLOG_ACTIVE_LEVEL SPDLOG_LEVEL_DEBUG
#endif
#include "spdlog/spdlog.h"


inline bool isFreespace(float x, float y, std::shared_ptr<Map> mp)
{
    int xCell =int(x), yCell =int(y);
    return (mp->data[yCell][xCell] < 0.5 && mp->data[yCell][xCell]> 0.0); 
}

ParticleFilter::ParticleFilter(const size_t _numParticles, std::shared_ptr<Map> mp) : numParticles(_numParticles)
{
    
    std::uniform_real_distribution<double> distX(mp->minX,mp->maxX), distY(mp->minY,mp->maxY), distTheta(-180.0,180.0);
    
    size_t numGenerated=0;
	SPDLOG_INFO("numparticles {}",numParticles);
    while(numGenerated < numParticles)
    {
        double x = distX(generator), y = distY(generator), theta = distTheta(generator);
        
        if(isFreespace(x,y,mp))
        {
            particles.emplace_back(Particle{ Pose2D{x, y, theta}, 1/double(numParticles)});
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
    throw "Not implemented yet";
}
