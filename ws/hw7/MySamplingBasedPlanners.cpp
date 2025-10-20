# include "MySamplingBasedPlanners.h"
# include "collisionCheck.h"
#include "tools/Visualizer.h"
#include <math.h>

// Implement your PRM algorithm here
amp::Path2D MyPRM::plan(const amp::Problem2D& problem) {
    amp::Timer timer("MyPRM::plan");  // Start timing
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
    int num_samples = 1000;
    double connection_radius = 1;

    amp::Path generic_path = prm_planner.plan(
        problem.q_init,          // Eigen::VectorXd(2)
        problem.q_goal,          // Eigen::VectorXd(2)
        cspace_nd,
        n,                     // number of PRM samples
        r                      // connection radius
    );


    // convert  nD path to 2D path and return it
    prm_nodes.clear();
    prm_nodes = prm_planner.getAllNodes();

    //graph_prm.clear();
    graph_prm = prm_planner.getGraph();

    if (!generic_path.waypoints.empty()) {
        for (const Eigen::VectorXd& q : generic_path.waypoints) {
            path2D.waypoints.push_back(q.head<2>());
        }
    } else {
        //std::cerr << "[WARN] PRM returned an empty path!" << std::endl;
    }
    // Print the most recent timing
    //double elapsed_ms = amp::Profiler::getMostRecentProfile("MyPRM::plan", amp::TimeUnit::ms);
    //std::cout << "Elapsed time: " << timer.now(amp::TimeUnit::ms) << " ms";
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
    amp::Timer timer("MyRRT::plan");  // Start timing

    amp::Path2D path;
    // make cspace
    std::size_t n_cells = 1000;
    MyPointAgentCSConstructor cspace_constructor(n_cells);
    std::unique_ptr<amp::GridCSpace2D> grid_cspace = cspace_constructor.construct(problem);
    amp::ConfigurationSpace2D& cspace2d = *grid_cspace;
    auto [x_min, x_max] = cspace2d.x0Bounds();
    auto [y_min, y_max] = cspace2d.x1Bounds();

    
    //std::cout << "Starting RRT!" << std::endl;

    // initialize tree
    std::vector<Eigen::Vector2d> nodes;
    std::shared_ptr<amp::Graph<double>> T = std::make_shared<amp::Graph<double>>();
    std::vector<int> parents;

    rrt_nodes.clear();
    rrt_graph = std::make_shared<amp::Graph<double>>();  // Use rrt_graph instead of T


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
    rrt_nodes.push_back(problem.q_init);  // Sync with rrt_nodes
    parents.push_back(-1);


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
            rrt_nodes.push_back(q_new);
            nodes.push_back(q_new);
            parents.push_back(near);
            int new_idx = static_cast<int>(nodes.size() - 1);
            
            // add edges to rrt_graph only
            double w = (q_new - q_near).norm();
            rrt_graph->connect((uint32_t)near, (uint32_t)new_idx, w);
            rrt_graph->connect((uint32_t)new_idx, (uint32_t)near, w);

            if ((q_new - problem.q_goal).norm() <= epsilon) {
                if (edge_collision_free(q_new, problem.q_goal)) {
                    nodes.push_back(problem.q_goal);
                    rrt_nodes.push_back(problem.q_goal);  // Sync with rrt_nodes
                    parents.push_back(new_idx);
                    goal_idx = static_cast<int>(nodes.size() - 1);
                    
                    double w2 = (problem.q_goal - q_new).norm();
                    rrt_graph->connect((uint32_t)new_idx, (uint32_t)goal_idx, w2);
                    rrt_graph->connect((uint32_t)goal_idx, (uint32_t)new_idx, w2);

                    found = true;
                    //std::cout << "[DEBUG] Goal found! Running A*" << std::endl;
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
        //std::cout << "[DEBUG] Unable to find goal, using nearest sample" << std::endl;
        nearest = nearest_index(problem.q_goal);
    } else {
        nearest = goal_idx;
    }

    // 3. run A* to find shortest path

    // set up shortest problem - use rrt_graph instead of T
    amp::ShortestPathProblem rrt_problem;
    rrt_problem.graph = rrt_graph;  // Use rrt_graph here
    rrt_problem.init_node = 0;
    rrt_problem.goal_node = nearest;

    amp::SearchHeuristic zeroHeuristic;

    std::unordered_map<amp::Node, Eigen::VectorXd> nodeStates;
    for (size_t i = 0; i < nodes.size(); i++) {
        nodeStates[i] = nodes[i];
    }
    EuclideanHeuristic euclidHeuristic(problem.q_goal, nodeStates);

    MyAStarAlgo algo;
    MyAStarAlgo::GraphSearchResult Astar_result = algo.search(rrt_problem, euclidHeuristic);

    // 4. reconstruct path
    std::list<amp::Node> node_path = Astar_result.node_path;
    if (Astar_result.success) {
        for (const amp::Node& n : Astar_result.node_path) {
            path.waypoints.push_back(nodes[n]);
        }
    } else {
        std::cerr << "RRT: no path found!" << std::endl;
    }

    
    
    if(smooth && Astar_result.success){
        // implement path smoothing
        int smoothPasses = 1000; // number of times to smooth
        
        //std::cout << "[DEBUG] Starting Path Smoothing" << std::endl;
        //std::cout << "[DEBUG] Number of waypoints before smoothing: " << (int)path.waypoints.size() << std::endl;
        // try smooth 1
        for(int pass = 0; pass<smoothPasses; pass++){
            int n = (int)path.waypoints.size();
            // select i and j randomly from list of waypoints (0,1,2,...n)
            int i = rand() % (n-1);
            int j = i+1+rand() % (n-i-1);
            // attempt to connect node i to node j
            const Eigen::Vector2d& q_i = path.waypoints[i];
            const Eigen::Vector2d& q_j = path.waypoints[j];

            bool collision_free = true;
            const double resolution = 0.2; // step size for collision checking
            double dist = (q_j - q_i).norm();
            int steps = std::max(2, (int)std::ceil(dist / resolution));

            for (int k = 0; k <= steps; k++) {
                double t = (double)k / (double)steps;
                Eigen::Vector2d interp = q_i + t * (q_j - q_i);
                if (cspace2d.inCollision(interp(0), interp(1))) {
                    collision_free = false;
                    break;
                }
            }
            // if yes, remove nodes inbetween i and j
            if (collision_free && (j > i + 1)) {
                path.waypoints.erase(path.waypoints.begin() + i + 1,
                                     path.waypoints.begin() + j);
            }
        }
        //std::cout << "[DEBUG] Smoothing complete. Final waypoint count: "
        //          << path.waypoints.size() << std::endl;
    }

    //std::cout << "Elapsed time: " << timer.now(amp::TimeUnit::ms) << " ms" << std::endl;
    //std::cout << "[DEBUG] Total RRT nodes: " << rrt_nodes.size() << std::endl;
    return path;

}

// Add this debugging function to your genericPRM class
void debugEdgeCollision(const Eigen::VectorXd& node_i, const Eigen::VectorXd& node_j, 
                    const amp::ConfigurationSpace& cspace) {
    std::cout << "\n=== Edge Collision Debug ===" << std::endl;
    std::cout << "Node i: (" << node_i(0) << ", " << node_i(1) << ")" << std::endl;
    std::cout << "Node j: (" << node_j(0) << ", " << node_j(1) << ")" << std::endl;
    std::cout << "Node i collision: " << cspace.inCollision(node_i) << std::endl;
    std::cout << "Node j collision: " << cspace.inCollision(node_j) << std::endl;
    std::cout << "Dist i->j: " << (node_i-node_j).norm() << std::endl;
    
    double edge_length = (node_i - node_j).norm();
    std::cout << "Edge length: " << edge_length << std::endl;
    
    // Test with different resolutions
    std::vector<double> resolutions = {0.5, 0.2, 0.1, 0.05};
    
    for (double res : resolutions) {
        int steps = std::max(2, (int)std::ceil(edge_length / res));
        int collisions = 0;
        
        std::cout << "\nResolution: " << res << ", Steps: " << steps << std::endl;
        
        for (int k = 0; k <= steps; k++) {
            double t = (double)k / (double)steps;
            Eigen::VectorXd interp = node_i + (node_j - node_i) * t;
            bool col = cspace.inCollision(interp);
            if (col) collisions++;
            if (k % 5 == 0 || col) {  // Print every 5th point or collisions
                std::cout << "  Step " << k << " (t=" << t << "): (" << interp(0) << ", " << interp(1) << ") -> " << col << std::endl;
            }
        }
        std::cout << "  Total collisions: " << collisions << " / " << (steps + 1) << std::endl;
    }
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
    //std::cout << "[DEBUG] Valid samples: " << valid_samples << " / " << num_samples << std::endl;
    //std::cout << "[DEBUG] all_nodes size: " << all_nodes.size() << std::endl;


    // 2. build graph

    graph_prm = std::make_shared<amp::Graph<double>>();
    int edge_created = 0;
    std::vector<int> edges_per_node(nodes.size(), 0);

    int max_edges_per_node_pair = (int)num_samples*0.05; // tune this for a more or less connected graph


    // add edge between two nodes
    for(size_t i = 0; i < nodes.size(); ++i) {
        for(size_t j = i+1; j < nodes.size(); ++j) {
            if ((nodes[i] - nodes[j]).norm() <= connection_radius) {
                // make sure nodes arent too far apart
                // skip if either node already has max edges
                if (edges_per_node[i] >= max_edges_per_node_pair || edges_per_node[j] >= max_edges_per_node_pair)
                    continue;
                bool collision_free = true;
                const double collision_check_resolution = 0.1; // Fixed step size

                double edge_length = (nodes[i] - nodes[j]).norm();

                int steps = std::max(2, (int)std::ceil(edge_length / collision_check_resolution));
                // DEBUG: Uncomment to see what's happening on first few edges

                //if (edge_created < 5) {
                //     debugEdgeCollision(nodes[i], nodes[j], cspace);
                //}

                for(int k = 0; k <= steps; k++){ 
                    double t = (double)k / (double)steps;
                    Eigen::VectorXd interp = nodes[i] + (nodes[j]-nodes[i])*t;
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
    //std::cout << "[DEBUG] Edges created: " << edge_created << std::endl;

    // Use a larger radius for start/goal connections to ensure they get connected
    auto connect_to_nearest = [&](int node_idx) {
        int connections = 0;
        double expanded_radius = connection_radius * 1.0; // Use larger radius for start/goal
        
        for (size_t i = 0; i < nodes.size(); i++) { 
            if (i == (size_t)node_idx) continue; // Skip self
            
            double dist = (nodes[node_idx] - nodes[i]).norm();
            if (dist <= expanded_radius) {
                bool collision_free = true;
                int steps = num_samples*0.5;
                for(int k = 0; k < steps; k++){
                    Eigen::VectorXd interp = nodes[node_idx] + (nodes[i]-nodes[node_idx])*(k/(double)steps);
                    if (cspace.inCollision(interp)) {
                        collision_free = false;
                        break;
                    }
                }
                
                if (collision_free) {
                    graph_prm->connect((uint32_t)node_idx, (uint32_t)i, dist);
                    graph_prm->connect((uint32_t)i, (uint32_t)node_idx, dist);
                    connections++;
                    edge_created++;
                }
            }
        }
        //std::cout << "[DEBUG] Connected node " << node_idx << " to " << connections << " nodes" << std::endl;
    };

    connect_to_nearest(0); // connect start
    connect_to_nearest(1); // connect goal
    //std::cout << "[DEBUG] Total edges after start/goal connection: " << edge_created << std::endl;

        

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
    // Fix 1: In genericPRM::plan, after A* search:
    if (Astar_result.success) {
        //std::cout << "[DEBUG] A* found path with " << Astar_result.node_path.size() << " nodes" << std::endl;
        for (const amp::Node& n : Astar_result.node_path) {
            path.waypoints.push_back(nodes[n]);
        }
        
        // ONLY do path smoothing if we actually have waypoints
        
        smooth = 1;
        if(smooth && !path.waypoints.empty()) {
            int smoothPasses = 1000;
            //std::cout << "[DEBUG] Starting Path Smoothing" << std::endl;
            //std::cout << "[DEBUG] Number of waypoints before smoothing: " << (int)path.waypoints.size() << std::endl;
            
            for(int pass = 0; pass < smoothPasses; pass++) {
                int n = (int)path.waypoints.size();
                if (n < 3) break;  // Need at least 3 waypoints to smooth
                
                int i = rand() % (n - 1);
                int j = i + 1 + rand() % (n - i - 1);
                
                const Eigen::Vector2d& q_i = path.waypoints[i];
                const Eigen::Vector2d& q_j = path.waypoints[j];

                bool collision_free = true;
                const double resolution = 0.2;
                double dist = (q_j - q_i).norm();
                int steps = std::max(2, (int)std::ceil(dist / resolution));

                for (int k = 0; k <= steps; k++) {
                    double t = (double)k / (double)steps;
                    Eigen::Vector2d interp = q_i + t * (q_j - q_i);
                    if (cspace.inCollision(interp)) {
                        collision_free = false;
                        break;
                    }
                }
                
                if (collision_free && (j > i + 1)) {
                    path.waypoints.erase(path.waypoints.begin() + i + 1,
                                        path.waypoints.begin() + j);
                }
            }
            //std::cout << "[DEBUG] Smoothing complete. Final waypoint count: "
            //        << path.waypoints.size() << std::endl;
        }
        

    } else {
        //std::cerr << "[ERROR] PRM: A* search failed! Returning empty path." << std::endl;
        // path is already empty, just return it
    }

    return path;
}


