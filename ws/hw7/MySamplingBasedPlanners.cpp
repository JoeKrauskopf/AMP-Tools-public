# include "MySamplingBasedPlanners.h"
# include "collisionCheck.h"
#include "tools/Visualizer.h"
#include <math.h>

// Implement your PRM algorithm here
amp::Path2D MyPRM::plan(const amp::Problem2D& problem) {
    amp::Path2D path2D;
    // make cspace
    std::size_t n_cells = 1000;
    MyPointAgentCSConstructor cspace_constructor(n_cells);
    std::unique_ptr<amp::GridCSpace2D> grid_cspace = cspace_constructor.construct(problem);
    amp::ConfigurationSpace2D& cspace2d = *grid_cspace;

    // verify cspace is generated correctly
    //amp::Visualizer::makeFigure(*grid_cspace);

    // convert cspace2d to cspace
    CSpace2DtoND cspace_nd(cspace2d);  // convert 2D cspace to nD interface
    /*
    std::cout << "[DEBUG MyPRM] Bounds before passing to plan: [" << cspace_nd.lowerBounds()(0) << ", " 
                << cspace_nd.upperBounds()(0) << "] x [" << cspace_nd.lowerBounds()(1) << ", " 
                << cspace_nd.upperBounds()(1) << "]" << std::endl;
    */
    // feed cspace to generic planner
    genericPRM prm_planner;
    int num_samples = 2000;
    double connection_radius = 5.0;

    amp::Path generic_path = prm_planner.plan(
        problem.q_init,          // Eigen::VectorXd(2)
        problem.q_goal,          // Eigen::VectorXd(2)
        cspace_nd,
        num_samples,                     // number of PRM samples
        connection_radius                      // connection radius
    );


    // convert  nD path to 2D path and return it
    prm_nodes.clear();
    prm_nodes = prm_planner.getAllNodes();

    //graph_prm.clear();
    graph_prm = prm_planner.getGraph();

    for (const Eigen::VectorXd& q : generic_path.waypoints) {
        path2D.waypoints.push_back(q.head<2>());  // take first 2 entries
    }
    return path2D;
}



struct EuclideanHeuristic : public amp::SearchHeuristic {
    Eigen::VectorXd goal;
    const std::unordered_map<amp::Node, Eigen::VectorXd>& states;

    EuclideanHeuristic(const Eigen::VectorXd& goal_,
                       const std::unordered_map<amp::Node, Eigen::VectorXd>& states_)
        : goal(goal_), states(states_) {}

    double operator()(amp::Node node) const override {
        auto it = states.find(node);
        if (it == states.end()) return 0.0; // fallback
        return (it->second - goal).norm();
    }
};


// Implement your RRT algorithm here
amp::Path2D MyRRT::plan(const amp::Problem2D& problem) {
    amp::Path2D path;
    // make cspace
    std::size_t n_cells = 1000;
    MyPointAgentCSConstructor cspace_constructor(n_cells);
    std::unique_ptr<amp::GridCSpace2D> grid_cspace = cspace_constructor.construct(problem);
    amp::ConfigurationSpace2D& cspace2d = *grid_cspace;
    auto [x_min, x_max] = cspace2d.x0Bounds();
    auto [y_min, y_max] = cspace2d.x1Bounds();

    // RRT parms:
    const int max_iterations = 100000;
    const double step_size = 2;
    const double epsilon = 0.5; // goal tolerance
    const double rho = 0.05; // goal bias
    
    std::cout << "Starting RRT!" << std::endl;

    // initialize tree
    std::vector<Eigen::Vector2d> nodes;
    std::shared_ptr<amp::Graph<double>> T = std::make_shared<amp::Graph<double>>();
    std::vector<int> parents;

    // Helper functions
    auto sample_random = [&]() -> Eigen::Vector2d {
        Eigen::Vector2d q;
        q(0) = x_min + (x_max - x_min) * ((double)rand() / RAND_MAX);
        q(1) = y_min + (y_max - y_min) * ((double)rand() / RAND_MAX);
        return q;
    };

    auto nearest_index = [&](const Eigen::Vector2d& q) -> int {
        int nearest = 0;
        double best_dist = std::numeric_limits<double>::infinity();
        for (int i = 0; i < (int)nodes.size(); i++) {
            double d = (nodes[i] - q).norm();
            if (d < best_dist) {
                best_dist = d;
                nearest = i;
            }
        }
        return nearest;
    };

    auto steer = [&](const Eigen::Vector2d& from, const Eigen::Vector2d& to) -> Eigen::Vector2d {
        Eigen::Vector2d diff = to - from;
        double dist = diff.norm();
        if (dist < step_size) return to;
        return from + diff * (step_size / dist);
    };

    auto edge_collision_free = [&](const Eigen::Vector2d& a, const Eigen::Vector2d& b) -> bool {
        const double edge_resolution = 0.2; // resolution for collision checking along an edge
        double dist = (b - a).norm();
        int steps = std::max(2, (int)std::ceil(dist / edge_resolution));
        for (int i = 0; i <= steps; ++i) {
            double t = (double)i / (double)steps;
            Eigen::Vector2d interp = a + t * (b - a);
            // call the 2-arg inCollision
            if (cspace2d.inCollision(interp(0), interp(1)))
                return false;
        }
        return true;
    };

    bool found = false;
    int goal_idx = -1;

    

    // create root at q_init
    nodes.push_back(problem.q_init);
    //nodes.push_back(problem.q_goal);
    std::cout << "[DEBUG] q_init: " << problem.q_init.transpose() << std::endl;
    parents.push_back(-1); // root has no parents


    // while not solution found:
    for (int i = 0; i<max_iterations; i++){

        //std::cout << "Iteration: " << i << std::endl;

        // generate random sample q_rand
        Eigen::Vector2d q_rand;
        q_rand = sample_random();

        //std::cout << "[DEBUG] q_rand: " << q_rand.transpose() << std::endl;

        // q_near <- nearest configuration to T to q_rand wrt rho
        int near = nearest_index(q_rand);
        Eigen::Vector2d q_near = nodes[near];

        //std::cout << "[DEBUG] q_near: " << q_near.transpose() << std::endl;

        // path <- generate path from q_near to q_rand

        // steer
        Eigen::Vector2d q_new = steer(q_near, q_rand);

        // collision check
        if (edge_collision_free(q_near, q_new)){

            // add q_new to tree
            nodes.push_back(q_new);
            parents.push_back(near);
            int new_idx = static_cast<int>(nodes.size() - 1);
            // add edges to T (both directions)
            double w = (q_new - q_near).norm();
            T->connect((uint32_t)near, (uint32_t)new_idx, w);
            T->connect((uint32_t)new_idx, (uint32_t)near, w);

            //std::cout << "[DEBUG] Connected near to new" << std::endl;

            // check for goal proximity
            if ((q_new - problem.q_goal).norm() <= epsilon) {
                // verify q_new -> goal is free and add exact goal node
                if (edge_collision_free(q_new, problem.q_goal)) {
                    nodes.push_back(problem.q_goal);
                    parents.push_back(new_idx);
                    goal_idx = static_cast<int>(nodes.size() - 1);

                    double w2 = (problem.q_goal - q_new).norm();
                    T->connect((uint32_t)new_idx, (uint32_t)goal_idx, w2);
                    T->connect((uint32_t)goal_idx, (uint32_t)new_idx, w2);

                    found = true;
                    std::cout << "[DEBUG] Goal found! Running A*" << std::endl;
                    break;
                }
            }
        }
    }
    //std::cout << "Found: " << found << std::endl; 
    // fallback for not finding goal
    int nearest = -1;
    if (!found) {
        // try connecting the closest node to the goal
        std::cout << "[DEBUG] Unable to find goal, using nearest sample" << std::endl;
        nearest = nearest_index(problem.q_goal);
    } else {
        nearest = goal_idx;
    }

    // 3. run A* to find shortest path

    // set up shortest problem
    amp::ShortestPathProblem rrt_problem;
    rrt_problem.graph = T;
    rrt_problem.init_node = 0;
    rrt_problem.goal_node = nearest; // change this

    
    amp::SearchHeuristic zeroHeuristic; // always returns 0

    // setup Heuristic

    std::unordered_map<amp::Node, Eigen::VectorXd> nodeStates;
    for (size_t i = 0; i < nodes.size(); i++) {
        nodeStates[i] = nodes[i];
    }
    EuclideanHeuristic euclidHeuristic(problem.q_goal, nodeStates);

    /*
    for (size_t i = 0; i < nodes.size(); i++) {
        nodeStates[i] = nodes[i];  // map node index to configuration
    }
    EuclideanHeuristic euclidHeuristic(goal_state, nodeStates); // fix this call
    */

    // call A*
    MyAStarAlgo algo;
    //MyAStarAlgo::GraphSearchResult Astar_result = algo.search(prm_problem, zeroHeuristic);
    MyAStarAlgo::GraphSearchResult Astar_result = algo.search(rrt_problem, euclidHeuristic);

    // 4. reconstruct path
    std::list<amp::Node> node_path = Astar_result.node_path;
    if (Astar_result.success) {
        for (const amp::Node& n : Astar_result.node_path) {
            path.waypoints.push_back(nodes[n]);  // map node index to configuration
        }
    } else {
        std::cerr << "RRT: no path found!" << std::endl;
    }
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

    // Cast to CSpace2DtoND to get proper bounds
    const CSpace2DtoND* cspace_ptr = dynamic_cast<const CSpace2DtoND*>(&collision_checker);
    if (!cspace_ptr) {
        std::cerr << "[ERROR] collision_checker is not a CSpace2DtoND!" << std::endl;
        return path;
    }
    const CSpace2DtoND& cspace = *cspace_ptr;

    /*
    std::cout << "[DEBUG CAST] After cast, bounds: [" << cspace.lowerBounds()(0) << ", " << cspace.upperBounds()(0) << "] x [" 
              << cspace.lowerBounds()(1) << ", " << cspace.upperBounds()(1) << "]" << std::endl;

    // Debug: Check init and goal states
    std::cout << "[DEBUG] Init state: (" << init_state(0) << ", " << init_state(1) << ")" << std::endl;
    std::cout << "[DEBUG] Goal state: (" << goal_state(0) << ", " << goal_state(1) << ")" << std::endl;
    std::cout << "[DEBUG] Init in collision? " << cspace.inCollision(init_state) << std::endl;
    std::cout << "[DEBUG] Goal in collision? " << cspace.inCollision(goal_state) << std::endl;
    std::cout << "[DEBUG] C-space bounds: [" << cspace.lowerBounds()(0) << ", " << cspace.upperBounds()(0) << "] x [" 
              << cspace.lowerBounds()(1) << ", " << cspace.upperBounds()(1) << "]" << std::endl;
    */

    // 1. sample cspace
    std::vector<Eigen::VectorXd> nodes;
    std::unordered_map<amp::Node, Eigen::VectorXd> nodeStates;

    //amp::SearchHeuristic basicHeuristic;
    nodes.push_back(init_state); //  add start node
    nodes.push_back(goal_state); // add end node
    int valid_samples = 0;

    for(int i = 0; i<num_samples; i++){
        // generate sample
        // Sample uniformly inside bounds
        Eigen::VectorXd sample(cspace.dimension());
        for (int d = 0; d < sample.size(); ++d){
            sample[d] = cspace.lowerBounds()[d] + 
                        (cspace.upperBounds()[d] - cspace.lowerBounds()[d]) * ((double) rand() / RAND_MAX);
        }
        // check if it collides in the cspace
        // NEED TO CHECK HOW inCollision IS WORKING FOR nD CSPACE
        if (!cspace.inCollision(sample)) {
            // if it does not add to list of valid nodes
            nodes.push_back(sample);
            all_nodes.push_back(sample.head<2>());  // Store 2D version for visualization

            valid_samples++;

        }
    }
    std::cout << "[DEBUG] Valid samples: " << valid_samples << " / " << num_samples << std::endl;
    std::cout << "[DEBUG] all_nodes size: " << all_nodes.size() << std::endl;


    // 2. build graph

    graph_prm = std::make_shared<amp::Graph<double>>();
    int edge_created = 0;
    std::vector<int> edges_per_node(nodes.size(), 0);

    int max_edges_per_node_pair = (int)num_samples*0.005; // tune this for a more or less connected graph

    // add edge between two nodes
    for(size_t i = 0; i < nodes.size(); ++i) {
        for(size_t j = i+1; j < nodes.size(); ++j) {
            if ((nodes[i] - nodes[j]).norm() <= connection_radius) {
                // make sure nodes arent too far apart
                // skip if either node already has max edges
                if (edges_per_node[i] >= max_edges_per_node_pair || edges_per_node[j] >= max_edges_per_node_pair)
                    continue;
                bool collision_free = true;
                int steps = num_samples*0.5; // scale to reduce compuation time for sparsely sampled spaces
                for(int k =0; k<steps; k++){
                    // test for collision along edge
                    Eigen::VectorXd interp = nodes[i] + (nodes[j]-nodes[i])*(k/(double)steps);
                    if (cspace.inCollision(interp)) {
                        collision_free = false;
                        break;
                    }
                }
                // if valid edge, add node pair and edge to graph
                if (collision_free) {
                    double dist = (nodes[i] - nodes[j]).norm();
                    graph_prm->connect((uint32_t)i, (uint32_t)j, dist);
                    graph_prm->connect((uint32_t)j, (uint32_t)i, dist);
                    edges_per_node[i]++;
                    edges_per_node[j]++;
                    edge_created++;
                }
            }
        }
    }
    std::cout << "[DEBUG] Edges created: " << edge_created << std::endl;

        

    // 3. run A* to find shortest path

    // set up shortest problem
    amp::ShortestPathProblem prm_problem;
    prm_problem.graph = graph_prm;
    prm_problem.init_node = 0;
    prm_problem.goal_node = 1;

    // call A*
    amp::SearchHeuristic zeroHeuristic; // always returns 0
    for (size_t i = 0; i < nodes.size(); i++) {
        nodeStates[i] = nodes[i];  // map node index to configuration
    }
    EuclideanHeuristic euclidHeuristic(goal_state, nodeStates);

    MyAStarAlgo algo;
    //MyAStarAlgo::GraphSearchResult Astar_result = algo.search(prm_problem, zeroHeuristic);
    MyAStarAlgo::GraphSearchResult Astar_result = algo.search(prm_problem, euclidHeuristic);

    // 4. reconstruct path
    std::list<amp::Node> node_path = Astar_result.node_path;
    if (Astar_result.success) {
        for (const amp::Node& n : Astar_result.node_path) {
            path.waypoints.push_back(nodes[n]);  // map node index to configuration
        }
    } else {
        std::cerr << "PRM: no path found!" << std::endl;
    }
    // TODO: TOGGLEABLE PATH SMOOTHING
    bool smooth = 0;
    
    if(smooth){
        // implement path smoothing

        // try smooth 1

    }


    return path;
}


