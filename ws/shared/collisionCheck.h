#ifndef COLLISIONCHECK_H
#define COLLISIONCHECK_H

#include "AMPCore.h"
#include <Eigen/Dense>

class collision {
public:
    struct Result {
        bool hit {false};
        int obstacleIdx {-1};
        int edgeHit{-1};
    };

    struct Check {
        bool hit {false};
        int edgeHit{-1};
    };

    Check collisionCheck(const Eigen::Vector2d& q, const amp::Polygon& obstacle);
    Result collisionCheckAll(const Eigen::Vector2d& q, const amp::Problem2D& problem);

private:
    bool collisionDetected = false;
};

#endif // COLLISIONCHECK_H