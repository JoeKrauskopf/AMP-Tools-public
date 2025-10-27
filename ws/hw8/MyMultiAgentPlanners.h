#pragma once

// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework headers
#include "hw/HW8.h"

// This is file is loaded from the shared/ directory
// Overwrite with your MySamplingBasedPlanners.h and MySamplingBasedPlanners.cpp from hw7
#include "MySamplingBasedPlanners.h" 



class MyCentralPlanner : public amp::CentralizedMultiAgentRRT {
    public:
        virtual amp::MultiAgentPath2D plan(const amp::MultiAgentProblem2D& problem) override; 
        size_t getTreeSize() const {
            return rrt_tree.rrt_nodes.size();
        }
        int max_iter =7500;
    private:
        MyRRT rrt_tree;
};


class MyDecentralPlanner : public MyRRT, public amp::DecentralizedMultiAgentRRT {
    public:
        virtual amp::MultiAgentPath2D plan(const amp::MultiAgentProblem2D& problem) override;
        size_t getTreeSize() const {
            return rrt_tree.rrt_nodes.size();
        }
        // Optional: you can add a getter to access the last elapsed time
        double getLastElapsedMs() const { return last_elapsed_ms; }
        int max_iter = 7500;
    
    private:
        MyRRT rrt_tree;
        double last_elapsed_ms = 0.0;
};