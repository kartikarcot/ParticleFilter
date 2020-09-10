#include<ParticleFilter.hpp>


inline bool isFreespace(float x, float y, std::shared_ptr<Map> mp)
{
    auto x_cell=int(x/mp->resolution), y_cell=int(y/mp->resolution);
    return true;//(mp->data[x_cell,y_cell] < 0.5 && mp->data[x_cell,y_cell] > 0.0); <- causing errors
}

ParticleFilter::ParticleFilter(const size_t num_particles, std::shared_ptr<Map> mp) : num_particles(num_particles)
{
    
    std::uniform_real_distribution<double> dist_x(mp->minX,mp->maxX), dist_y(mp->minY,mp->maxY), dist_theta(-180.0,180.0);
    
    size_t num_generated=0;
    while(num_generated <= num_particles)
    {
        auto x = dist_x(generator), y = dist_y(generator), theta = dist_theta(generator);
        
        if(isFreespace(x,y,mp))
        {
            particles.emplace_back(Particle{ Pose2D{x, y, theta}, 1/double(num_particles)});
            num_generated++;
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
