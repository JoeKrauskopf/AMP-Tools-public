#pragma once

#include "AMPCore.h"
#include "hw/HW2.h"
#include <Eigen/Dense>
#include "collisionCheck.h"

/// @brief Declare your bug algorithm class here. Note this class derives the bug algorithm class declared in HW2.h
class MyBugAlgorithm : public amp::BugAlgorithm {
    public:
        // Override and implement the bug algorithm in the plan method. The methods are declared here in the `.h` file
        virtual amp::Path2D plan(const amp::Problem2D& problem) override;
        
        // Add any other methods here...
        
    
    private:
        // Add any member variables here...
        Eigen::MatrixXd q_L;  // N×2 matrix
        Eigen::MatrixXd q_H;  // N×2 matrix
        Eigen::MatrixXd Q; // Nx2 matrix of all points travelled
        Eigen::MatrixXd QBoundary;
        char direction = 'L'; // define if robot is right [R] or left [L] turning TEST BOTH
        bool goalReached;
        bool collide;
        bool boundaryFollowing;
};