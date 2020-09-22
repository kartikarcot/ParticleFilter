#include<Utils.hpp>


inline bool isFreespace(float x, float y, std::shared_ptr<Map> mp)
{
    int xCell =int(x), yCell =int(y);
    return (mp->data[yCell][xCell] >= 0.0 && mp->data[yCell][xCell] <= 0.1); 
}

