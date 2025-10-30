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
    Result collisionCheckAllEnv(const Eigen::Vector2d& q, const amp::Environment2D& env);
    Check collisionCheckDiskAgent(const Eigen::Vector2d& q, const double radius, const amp::Polygon& obstacle);
    Result collisionCheckDiskAgentAllEnv(const Eigen::Vector2d& q, const double radius, const amp::Environment2D& env);
    Check collisionCheckDynamicAgent(const Eigen::Vector2d& q, const amp::DynamicAgent& agent, const double theta, const amp::Polygon& obstacle, const amp::Environment2D& env);
    Result collisionCheckDynamicAgentAllEnv(const Eigen::Vector2d& q, const amp::DynamicAgent& agent, const double theta, const amp::Environment2D& env);

private:
    bool collisionDetected = false;
};

#endif // COLLISIONCHECK_H