#pragma once

// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework headers
#include "hw/HW7.h"
#include "collisionCheck.h"
#include "MyAStar.h"
#include "MyCSConstructors.h"
#include <vector>
#include <memory>

class MyPRM : public amp::PRM2D {
    public:
        virtual amp::Path2D plan(const amp::Problem2D& problem) override; 
        std::vector<Eigen::Vector2d> getPRMNodes() const { return prm_nodes; }
        std::shared_ptr<amp::Graph<double>> getPRMGraph() const { return graph_prm; } 

    private:
        std::vector<Eigen::Vector2d> prm_nodes;  // Store all sampled nodes
        std::shared_ptr<amp::Graph<double>> graph_prm;  // graph member

};

class MyRRT : public amp::GoalBiasRRT2D {
    public:
        virtual amp::Path2D plan(const amp::Problem2D& problem) override; 
};

class genericPRM {
    public:
        amp::Path plan(
            const Eigen::VectorXd& init_state, 
            const Eigen::VectorXd& goal_state, 
            const amp::ConfigurationSpace& collision_checker,
            int num_samples = 100,
            double connection_radius = 1.0
        );
        std::vector<Eigen::Vector2d> getAllNodes() const { return all_nodes; } // legacy
        std::shared_ptr<amp::Graph<double>> getGraph() const { return graph_prm; } 

    private:
        std::vector<Eigen::Vector2d> all_nodes;  // Store all sampled nodes
        std::shared_ptr<amp::Graph<double>> graph_prm;  // graph member



};   

// class to convert 2d to nd cspaces
struct CSpace2DtoND : public amp::ConfigurationSpace {
    const amp::ConfigurationSpace2D& c2d;
    Eigen::VectorXd lower, upper;

    CSpace2DtoND(const amp::ConfigurationSpace2D& c)
        : amp::ConfigurationSpace(Eigen::VectorXd::Zero(2), Eigen::VectorXd::Zero(2)),
          c2d(c),
          lower(2),
          upper(2)
    {
        // Get the actual bounds from the 2D cspace
        auto x0_bounds = c.x0Bounds();
        auto x1_bounds = c.x1Bounds();
        
        lower(0) = x0_bounds.first;
        lower(1) = x1_bounds.first;
        upper(0) = x0_bounds.second;
        upper(1) = x1_bounds.second;
        
        // Debug output to verify bounds
        /*
        std::cout << "[DEBUG CSpace2DtoND] Bounds set to: [" << lower(0) << ", " << upper(0) << "] x [" 
                  << lower(1) << ", " << upper(1) << "]" << std::endl;
        */
    }

    bool inCollision(const Eigen::VectorXd& q) const {
        return c2d.inCollision(q[0], q[1]);
    }

    const Eigen::VectorXd& lowerBounds() const { return lower; }
    const Eigen::VectorXd& upperBounds() const { return upper; }
    std::size_t dimension() const { return 2; }
};
