#include<Utils.hpp>


void normalizeAndShiftWeights(std::vector<double>& weights)
{
    double sum = 0 ;
	double minValue = *(std::min_element(weights.begin(), weights.end()));

	/* for (int i = 0; i < weights.size(); i++) */
	/* { */
	/* 	weights[i]-=(minValue-1e-6); */
	/* } */

    // for(auto &weight:weights)
    //     sum+=weight;

    for (auto &weight : weights)
    {
        weight = exp((weight));
        sum += weight;
    }

    for (auto &weight : weights)
    {
        weight /= sum;
    }
}
