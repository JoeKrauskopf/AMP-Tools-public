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
        std::vector<Eigen::Vector2d> prm_nodes;  // Store all sampled nodes
        std::shared_ptr<amp::Graph<double>> graph_prm;  // graph member
        std::vector<Eigen::Vector2d> all_nodes; // includes start, goal, and all sampled nodes

        int n = 200; // number of nodes to sample
        double r = 2; // connection radius

    private:
        

};

class MyRRT : public amp::GoalBiasRRT2D {
    public:

        struct Gamma
        {
            // could be alterted to be nxm matrix
            // each column is the waypoints of robot i
            std::vector<amp::Path2D> agent_paths; // one Path2D per previous agent
            std::vector<int> agent_idx;
            double elapsedTime;
        };
        struct RRTNode {
            Eigen::Vector2d pos; // 2D position
            double t;            // arrival time
            int parent_idx;      // parent in the tree
        };


        virtual amp::Path2D plan(const amp::Problem2D& problem) override;
        virtual amp::MultiAgentPath2D planMultiCoupled(const amp::MultiAgentProblem2D& problem);
        virtual amp::Path2D planMultiDeCoupled(const amp::MultiAgentProblem2D& problem, const MyRRT::Gamma gamma_prev, const int agent_idx);
        

        // For visualization
        std::vector<Eigen::Vector2d> rrt_nodes;                // All sampled nodes
        std::shared_ptr<amp::Graph<double>> rrt_graph;         // RRT tree edges 
        int max_iterations = 70000;
        double step_size = 0.5;
        double epsilon = 0.25; // goal tolerance
        double rho = 0.05; // goal bias
        bool smooth = 0;
        
        size_t getTreeSize() const {
            return rrt_nodes.size();
        }
        
    private:
        
};

class genericPRM {
    public:
        amp::Path plan(
            const Eigen::VectorXd& init_state, 
            const Eigen::VectorXd& goal_state, 
            const amp::ConfigurationSpace& collision_checker,
            int num_samples,
            double connection_radius
        );
        std::vector<Eigen::Vector2d> getAllNodes() const { return all_nodes; } // legacy
        std::shared_ptr<amp::Graph<double>> getGraph() const { return graph_prm; } 
        std::vector<Eigen::Vector2d> all_nodes;  // Store all sampled nodes
        std::shared_ptr<amp::Graph<double>> graph_prm;  // graph member
        bool smooth = 1;

    private:
        

};   

// class to convert 2d to nd cspaces
struct CSpace2DtoND : public amp::ConfigurationSpace {
    const amp::ConfigurationSpace2D& c2d;

    CSpace2DtoND(const amp::ConfigurationSpace2D& c)
        : amp::ConfigurationSpace(Eigen::VectorXd::Zero(2), Eigen::VectorXd::Zero(2)),
          c2d(c)
    {
        // Get the actual bounds from the 2D cspace
        auto x0_bounds = c.x0Bounds();
        auto x1_bounds = c.x1Bounds();
        
        // Set the base class member variables directly
        m_lower_bounds(0) = x0_bounds.first;
        m_lower_bounds(1) = x1_bounds.first;
        m_upper_bounds(0) = x0_bounds.second;
        m_upper_bounds(1) = x1_bounds.second;
    }

    /// @brief Check collision by delegating to the 2D cspace's grid
    /// @param q 2D configuration state [x0, x1]
    /// @return true if in collision (cell value = 1), false if free (cell value = 0)
    bool inCollision(const Eigen::VectorXd& q) const override {
        if (q.size() != 2) {
            throw std::runtime_error("CSpace2DtoND expects 2D states");
        }
        return c2d.inCollision(q(0), q(1));
    }
};

inline double l2Distance(const Eigen::VectorXd& a, const Eigen::VectorXd& b) {
    if (a.size() != b.size()) {
        throw std::runtime_error("Vectors must be the same size for L2 distance.");
    }
    return (a - b).norm();  // L2 norm
}

