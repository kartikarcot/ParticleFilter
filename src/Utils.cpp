#include<Utils.hpp>


void normalize_weights(std::vector<double>& weights)
{
    double sum = 0 ;
    for(auto &weight:weights)
        sum+=weight;
    for(auto &weight:weights)
        weight/=sum;
}
