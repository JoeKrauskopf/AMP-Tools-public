#pragma once

#include "AMPCore.h"
#include "hw/HW2.h"
#include <Eigen/Dense>
#include "collisionCheck.h"
#include <algorithm>

/// @brief Declare your bug algorithm class here. Note this class derives the bug algorithm class declared in HW2.h
class MyBugAlgorithm : public amp::BugAlgorithm {
    public:
        // Override and implement the bug algorithm in the plan method. The methods are declared here in the `.h` file
        virtual amp::Path2D plan(const amp::Problem2D& problem) override;
        
        virtual amp::Path2D planBUG2(const amp::Problem2D& problem);
        
    
     private:
        
        Eigen::Vector2d followBoundary(const Eigen::Vector2d& current,
                                const Eigen::Vector2d& previous,
                                const amp::Problem2D& problem,
                                collision& coll,
                                double stepSize,
                                double radius,
                                char direction);

        Eigen::Vector2d moveTowardLeavePoint(const Eigen::Vector2d& current,
                                            const Eigen::Vector2d& previous,
                                            const Eigen::Vector2d& leavePoint,
                                            const amp::Problem2D& problem,
                                            collision& coll,
                                            double stepSize,
                                            double radius,
                                            char direction);
        bool isOnMline(const Eigen::Vector2d& current,
                    const Eigen::Vector2d& start,
                    const Eigen::Vector2d& end,
                    double tol);
            
        // Member variables
        Eigen::Vector2d q;
        Eigen::Vector2d q_next;
        Eigen::Vector2d q_previous;
        Eigen::MatrixXd q_L;  // N×2 matrix
        Eigen::MatrixXd q_H;  // N×2 matrix
        Eigen::MatrixXd Q; // Nx2 matrix of all points travelled
        Eigen::MatrixXd QBoundary;
        Eigen::Vector2d currentTargetVertex;
        bool hasTargetVertex = false;
        char direction = 'L'; // define if robot is right [R] or left [L] turning TEST BOTH
        bool goalReached;
        bool collide;
        bool boundaryFollowing;
        bool loopCompleted;
        //bool atCorner;
        int lastBoundaryDirection = 0; // Track last boundary following direction
        int currentObstacleIdx;
};