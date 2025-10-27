#include "MyMultiAgentPlanners.h"
#include "hw/HW8.h"

amp::MultiAgentPath2D MyCentralPlanner::plan(const amp::MultiAgentProblem2D& problem) {
    amp::MultiAgentPath2D path;
    // choose coupled approach
    std::cout << "[DEBUG] Num Agents: " << problem.agent_properties.size() << std::endl;
    // Reset the RRT tree from previous runs
    rrt_tree.rrt_nodes.clear();
    rrt_tree.rrt_graph = std::make_shared<amp::Graph<double>>();


    // add a way of scaling rrt parameters based on the multiagent problem (num agents)
    int base_iterations = max_iter;    // iterations for 1 agent // 5000 WORKS FOR 1-6 AGENTS ~64/100 BENCHMARKS
    int n_agents = problem.agent_properties.size();

    // exponential scaling
    //rrt.max_iterations = static_cast<int>(base_iterations * std::pow(2.0, n_agents - 1));
    rrt_tree.max_iterations = base_iterations;
    //std::cout << "[DEBUG] Max Iterations: " << rrt.max_iterations << std::endl;

    path = rrt_tree.planMultiCoupled(problem); // run coupled multiagent planner

    std::vector<std::vector<Eigen::Vector2d>> collision_states;
    bool isValid = amp::HW8::check(path, problem, collision_states);
    int maxAttempts = 10000;
    int attempt = 0;
    while(!isValid){
        std::cout << "Attempt: " << attempt << " of " << maxAttempts << std::endl;

        rrt_tree.rrt_nodes.clear();
        rrt_tree.rrt_graph = std::make_shared<amp::Graph<double>>();

        path = rrt_tree.planMultiCoupled(problem); // run coupled multiagent planner
        isValid = amp::HW8::check(path, problem, collision_states);
        if(attempt >= maxAttempts){
            std::cout << "[WRN] Failed to find valid path after " << maxAttempts << " attempts." << std::endl;
            return path;
            break;
        }
        attempt++;
    }

    return path;
}


amp::MultiAgentPath2D MyDecentralPlanner::plan(const amp::MultiAgentProblem2D& problem) {
    amp::MultiAgentPath2D path;
    bool isValid = false;
    std::vector<std::vector<Eigen::Vector2d>> collision_states;
    int maxAttempts = 10000;
    int attempt = 0;

    double elapsed_ms = 0.0;                     // Reset elapsed time
    

    // choose priority based decoupled approach
    while(!isValid) {
        std::cout << "Attempt: " << attempt << " of " << maxAttempts << std::endl;
        // assume every agent moves from qi to qj in 1 second
        amp::Timer timer("MultiRRT_Benchmark"); // Start timer
    
        // put agents into a priority queue based on path lengths (shortest first)
        std::vector<std::pair<int,double>> agent_priority; // (index, distance)
        int n_agents = problem.agent_properties.size();
        int base_iterations = max_iter;    // iterations for 1 agent

        std::cout << "[DEBUG] Num Agents: " << problem.agent_properties.size() << std::endl;
        if(attempt == 0) {
            agent_priority.clear();

            for (int i = 0; i < n_agents; ++i) {
                double dist = (problem.agent_properties[i].q_goal - problem.agent_properties[i].q_init).norm();
                agent_priority.push_back({i, dist});
            }
            // sort shortest first
            std::sort(agent_priority.begin(), agent_priority.end(),
                    [](const auto& a, const auto& b){ return a.second < b.second; });
        } else{
            // randomize order
            agent_priority.clear();
            for (int i = 0; i < n_agents; ++i) {
                agent_priority.push_back({i, 0.0}); // distance not relevant here
            }

            // Shuffle order randomly
            std::random_device rd;
            std::mt19937 g(rd());
            std::shuffle(agent_priority.begin(), agent_priority.end(), g);
        }
        

        

        // first gamma is empty
        MyRRT::Gamma gamma_prev;

        std::vector<amp::Path2D> tmp_paths(n_agents);
        

        //for ever agent in queue
        for(const auto& p : agent_priority){
            int agent_idx = p.first;
            const auto& agent = problem.agent_properties[agent_idx];

            // 1. Plan path for this agent
            rrt_tree.rrt_nodes.clear();
            rrt_tree.rrt_graph = std::make_shared<amp::Graph<double>>();

            // exponential scaling
            rrt_tree.max_iterations = base_iterations;

            //rrt.max_iterations = static_cast<int>(base_iterations * std::pow(2.0, n_agents - 1));
            //std::cout << "[DEBUG] Max Iterations: " << rrt.max_iterations << std::endl;

            amp::Path2D agent_path = rrt_tree.planMultiDeCoupled(problem, gamma_prev, agent_idx);


            // Only append new waypoints for the current agent
            gamma_prev.agent_paths.push_back(agent_path); 
            gamma_prev.elapsedTime += agent_path.waypoints.size(); // or duration if time-parametrized
            gamma_prev.agent_idx.push_back(agent_idx);

            // Store agent path
            tmp_paths[agent_idx] = agent_path;  // store in correct slot
            std::cout << "[DEBUG] Finished planning agent: " << agent_idx << std::endl;
        }

        path.agent_paths = tmp_paths;
         
        // now check path
        isValid = amp::HW8::check(path, problem, collision_states);
        if (isValid) {
            // Record elapsed time for successful run only
            last_elapsed_ms = timer.now(amp::TimeUnit::ms);
            std::cout << "[INFO] Planner succeeded in " << elapsed_ms << " ms." << std::endl;
            break;
        }
        // iterate attempts
        if(attempt >= maxAttempts){
            std::cout << "[WRN] Failed to find valid path after " << maxAttempts << " attempts." << std::endl;
            return path;
            break;
        }
        attempt++;

    }
    return path;

}