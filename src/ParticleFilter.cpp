#include<ParticleFilter.hpp>
#include <random>


#define PI 3.14159

void normalize_weights(std::vector<double>& weights)
{
    double sum =0 ;
    for(auto &weight:weights)
        sum+=weight;
    for(auto &weight:weights)
        weight/=sum;
}

inline bool isFreespace(float x, float y, std::shared_ptr<Map> mp)
{
    return (mp->at(x,y) >= 0.0 && mp->at(x,y) <= 0.5); 
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
    
    normalize_weights(weights);
	
    std::discrete_distribution<int> distribution(weights.begin(), weights.end());
	std::vector<Pose2D> newParticles(numParticles);
	for (auto &newParticle : newParticles)
	{
		newParticle = particles[distribution(generator)];
	}
	particles = std::move(newParticles);
	return;
}
