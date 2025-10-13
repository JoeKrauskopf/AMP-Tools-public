# include "MySamplingBasedPlanners.h"
# include "collisionCheck.h"

// Implement your PRM algorithm here
amp::Path2D MyPRM::plan(const amp::Problem2D& problem) {
    amp::Path2D path;
    // make cspace

    // feed cspace to generic planner

    // convert  nD path to 2D path and return it


    path.waypoints.push_back(problem.q_init);
    path.waypoints.push_back(problem.q_goal);
    return path;
}


// Implement your RRT algorithm here
amp::Path2D MyRRT::plan(const amp::Problem2D& problem) {
    amp::Path2D path;
    path.waypoints.push_back(problem.q_init);
    path.waypoints.push_back(problem.q_goal);
    return path;
}

amp::Path genericPRM::plan(
            const Eigen::VectorXd& init_state, 
            const Eigen::VectorXd& goal_state, 
            const amp::ConfigurationSpace& collision_checker,
            int num_samples,
            double connection_radius
) {
    amp::Path path;
    collision coll;
    collision::Result result;

    // 1. sample cspace
    std::vector<Eigen::VectorXd> nodes;
    nodes.push_back(q_init); //  add start node
    nodes.push_back(q_goal); // add end node

    for(int i = 0; i<num_samples; i++){
        // generate sample
        // Sample uniformly inside bounds
        Eigen::VectorXd sample(collision_checker.dimension());
        for (int d = 0; d < sample.size(); ++d){
            sample[d] = collision_checker.lowerBounds()[d] + 
                        (collision_checker.upperBounds()[d] - collision_checker.lowerBounds()[d]) * ((double) rand() / RAND_MAX);
        }
        // check if it collides in the cspace
        if(collision_checker.dimension() == 2) {
            // handle 2d case for now
            
        }
        // if it does not add to list of valid nodes
    }

    // 2. build graph
        // add edge between two nodes
        // collision check the edge
        // if valid edge, add node pair and edge to graph

    // 3. run A* to find shortest path


    return path;
}