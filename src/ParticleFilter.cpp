#include<ParticleFilter.hpp>
#include <random>
#include "config.hpp"

#define PI 3.14159


inline bool isFreespace(
		const float &x, 
		const float &y, 
		const std::shared_ptr<Map> &mp)
{
    return (mp->valid(x,y) && mp->at(x,y) >= 0.0 && mp->at(x,y) <= mp->freespaceThreshold); 
}


ParticleFilter::ParticleFilter(
					const size_t _numParticles, 
					const std::shared_ptr<Map> &mp,
					const double &_posVar,
					const double &_thetaVar,
					const int &seed) : 
						numParticles(_numParticles),
						weights(std::vector<double>(_numParticles, 1/double(_numParticles))),
						posVar(_posVar),
						thetaVar(_thetaVar)
{
    
    std::uniform_real_distribution<double> distX(mp->minX,mp->maxX), distY(mp->minY,mp->maxY), distTheta(-PI, PI);
    std::default_random_engine generator(seed);
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
	/* SPDLOG_DEBUG("The weights after normalizing are"); */
	/* for (const auto &w : weights) */
	/* { */
	/* 	std::cout<<w<<" "; */
	/* } */
	/* std::cout<<std::endl; */
	normalizeAndShiftWeights(weights);
	// std::normal_distribution<double> x_noise(0.0,posVar), y_noise(0.0,posVar), theta_noise(0.0,thetaVar);
	// std::random_device rd;
	std::default_random_engine generator(SEED);
    // normalize_weights(weights);
    std::discrete_distribution<int> distribution(weights.begin(), weights.end());
	std::vector<Pose2D> newParticles(numParticles);
	for (auto &newParticle : newParticles)
	{
		newParticle = particles[distribution(generator)];
		// newParticle.x += x_noise(generator);
		// newParticle.y += y_noise(generator);
		// newParticle.theta += theta_noise(generator);
	}
	particles = std::move(newParticles);
	return;
}

void ParticleFilter::lowVarianceResample(const std::shared_ptr<Map> &mp, const int &seed)
{
	std::vector<double> cumulativeWeights(numParticles);
	double stepSize = ((double)1/numParticles);
	std::random_device rd;
	std::default_random_engine generator(seed);
	std::uniform_real_distribution<> distribution(0, stepSize);
	double startVal = distribution(generator), curVal = 0;
	std::vector<Pose2D> newParticles(numParticles);
	// std::normal_distribution<double> x_noise(0.0,posVar), y_noise(0.0,posVar), theta_noise(0.0,thetaVar);

	// normalize the weights
	normalizeAndShiftWeights(weights);
	// calculate the cumulativeWeightvector
	cumulativeWeights[0] = weights[0];
	for (int i = 1; i < numParticles; i++)
		cumulativeWeights[i] = cumulativeWeights[i-1] + weights[i];

	// sample values and get indices
	for (int i = 0; i < numParticles; i++)
	{
		curVal = startVal + i * (stepSize);
		auto upperBoundIter = std::upper_bound(cumulativeWeights.begin(), cumulativeWeights.end(), curVal);
		int index = std::max((long)0, std::distance(cumulativeWeights.begin(), upperBoundIter));
		// calculate the new partcle with some noise
		newParticles[i] = particles[index];
		auto newX = newParticles[i].x;// + x_noise(generator);
		auto newY = newParticles[i].y;// + y_noise(generator);
		if(isFreespace(newX,newY,mp))
		{
			newParticles[i].x = newX;
			newParticles[i].y = newY;
		}
		// newParticles[i].theta += theta_noise(generator);
	}
	particles = std::move(newParticles);
	return;
}

void ParticleFilter::lowVarianceResampleTest(const std::shared_ptr<Map> &mp)
{
	std::vector<double> cumulativeWeights(numParticles);
	double stepSize = ((double)1/numParticles);
	std::random_device rd;
	std::default_random_engine generator(rd());
	std::uniform_real_distribution<> distribution(0, stepSize);
	double startVal = distribution(generator), curVal = 0;
	std::vector<Pose2D> newParticles(numParticles);
	std::normal_distribution<double> x_noise(0.0,posVar), y_noise(0.0,posVar), theta_noise(0.0,thetaVar);

	// normalize the weights
	normalizeAndShiftWeights(weights);
	// calculate the cumulativeWeightvector
	cumulativeWeights[0] = weights[0];
	for (int i = 1; i < numParticles; i++)
		cumulativeWeights[i] = cumulativeWeights[i-1] + weights[i];
	std::unordered_map<int,int> countMap;
	// sample values and get indices
	for (int i = 0; i < numParticles; i++)
	{
		curVal = startVal + i * (stepSize);
		auto upperBoundIter = std::upper_bound(cumulativeWeights.begin(), cumulativeWeights.end(), curVal);
		int index = std::max((long)0, std::distance(cumulativeWeights.begin(), upperBoundIter));
		countMap[index]++;
		// calculate the new partcle with some noise
		
		newParticles[i] = particles[index];
		
		
		newParticles[i].theta += theta_noise(generator);
		auto newX = newParticles[i].x + x_noise(generator);
		auto newY = newParticles[i].y + y_noise(generator);
		if(isFreespace(newX,newY,mp))
		{
			newParticles[i].x = newX;
			newParticles[i].y = newY;
		}
		
		
	}

	for (const auto &it : countMap)
		SPDLOG_DEBUG("The index {} was sampled {} times", it.first, it.second);

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
