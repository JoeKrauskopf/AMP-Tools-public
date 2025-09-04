#include "AMPCore.h"
#include <cmath>
#include <algorithm>

class collision {
    public:
        bool collisionCheck(const Eigen::Vector2d q, const amp::Polygon& obstacle);
        bool collisionCheckAll(const Eigen::Vector2d q, const amp::Problem2D& problem);
    private:
        bool collisionDetected = false;
};