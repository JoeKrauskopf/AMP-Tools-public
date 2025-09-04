#pragma once

#include "AMPCore.h"
#include "hw/HW2.h"

/// @brief Declare your bug algorithm class here. Note this class derives the bug algorithm class declared in HW2.h
class MyBugAlgorithm : public amp::BugAlgorithm {
    public:
        // Override and implement the bug algorithm in the plan method. The methods are declared here in the `.h` file
        virtual amp::Path2D plan(const amp::Problem2D& problem) override;

        // Add any other methods here...
    
    private:
        // Add any member variables here...
        Eigen::Vector2d q_H; // Hit point
        Eigen::Vector2d q_L; // Leave point
        bool followingObstacle = false; // flag to check if in obstacle following mode
};