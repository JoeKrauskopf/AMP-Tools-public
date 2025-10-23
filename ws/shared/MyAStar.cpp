#include "MyAStar.h"
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <functional>
#include <limits>
#include <vector>
#include <iostream>

// Implement the search method for the A* algorithm
MyAStarAlgo::GraphSearchResult MyAStarAlgo::search(const amp::ShortestPathProblem& problem, const amp::SearchHeuristic& heuristic) {
    bool debug = 0;
    if(debug) {
        std::cout << "Starting A* Graph Search: Init --> goal | " << problem.init_node << " --> " << problem.goal_node << std::endl;
    }
    GraphSearchResult result = {false, {}, 0.0}; // initialize the results object

    auto G = problem.graph;
    amp::Node root = problem.init_node;
    amp::Node goal = problem.goal_node;

    // make open list of nodes O
    std::priority_queue<NodeCost, std::vector<NodeCost>, std::greater<NodeCost>> O;
    int iteration_count = 0;
    O.push({root, heuristic(root)}); // start at root

    std::unordered_map<amp::Node, amp::Node> came_from; // backpointers
    std::unordered_map<amp::Node, double> g_cost;        // cost from start to node
    std::unordered_set<amp::Node> C;            // visited nodes

    g_cost[root] = 0.0;

    // while O is not empty
    while(!O.empty()){
        // pick best node from O s.t. estimated cost of best node <= estimated cost of all available nodes
        iteration_count++;  // count this iteration
        NodeCost current = O.top();
        O.pop(); // select next node in open list

        // if best node is goal node, exit and return path
        if(current.node == goal) {
            // reconstruct path
            amp::Node n = goal;
            std::vector<amp::Node> path;
            while (came_from.find(n) != came_from.end()) {
                path.push_back(n);
                n = came_from[n];
            }
            path.push_back(root);
            std::reverse(path.begin(), path.end());
            result.node_path = std::list<amp::Node>(path.begin(), path.end());;
            result.path_cost = g_cost[goal];
            result.success = true;
            if(debug) {
                result.print();
            
                std::cout << "A* completed in " << iteration_count << " iterations." << std::endl;
            }
            return result;

        }

        if (C.find(current.node) != C.end()) continue;
        C.insert(current.node); // add best node to close list

        const auto& neighbors = G->children(current.node); // find children
        const auto& edges = G->outgoingEdges(current.node); // find outgoing edges

        for (size_t i = 0; i < neighbors.size(); ++i) {
            amp::Node neighbor = neighbors[i];
            double edge_cost = edges[i];
            double tentative_g = g_cost[current.node] + edge_cost;

            if (g_cost.find(neighbor) == g_cost.end() || tentative_g < g_cost[neighbor]) {
                g_cost[neighbor] = tentative_g;
                double f = tentative_g + heuristic(neighbor);
                O.push({neighbor, f});
                came_from[neighbor] = current.node;
            }
        }

        // expand best node: for all x in set of nodes adjacent to n that are not in C
        //      if x does not exist in O
        //          add x to O
        //      else if g(best node) + (c(best node, x)) < g(x) then
        //          update x's backpointer to point to best node
        // 

    }


        
    

    




    result.node_path.push_back(problem.goal_node);
    result.path_cost += 1.0;
    if(debug) {
        //result.print();
    }
    return result;
}

