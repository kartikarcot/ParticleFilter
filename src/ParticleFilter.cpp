#include<ParticleFilter.hpp>


inline bool is_freespace(float x, float y, std::shared_ptr<Map> mp)
{
    auto x_cell=int(x/mp->resolution), y_cell=int(y/mp->resolution);
    return true;//(mp->data[x_cell,y_cell] < 0.5 && mp->data[x_cell,y_cell] > 0.0);
}

ParticleFilter::ParticleFilter(const size_t num_particles, std::shared_ptr<Map> mp) : num_particles(num_particles)
{
    std::default_random_engine gen;
    std::uniform_real_distribution<double> dist_x(mp->minX,mp->maxX), dist_y(mp->minY,mp->maxY), dist_theta(-180.0,180.0);
    
    size_t num_generated=0;
    while(num_generated<num_particles)
    {
        auto x = dist_x(gen), y = dist_y(gen), theta = dist_theta(gen);
        
        if(is_freespace(x,y,mp))
        {
            particles.emplace_back(Particle{ x, y, theta, 1/double(num_particles)});
            num_generated++;
        }
    }
    
}

State ParticleFilter::predict(State& x_t)
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