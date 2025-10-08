#pragma once

// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework headers
#include "hw/HW6.h"

class MyAStarAlgo : public amp::AStar {
    public:
        virtual GraphSearchResult search(const amp::ShortestPathProblem& problem, const amp::SearchHeuristic& heuristic) override;
    struct NodeCost {
        amp::Node node;
        double f_cost; // f(n) = g(n) + h(n)
        bool operator>(const NodeCost& other) const { return f_cost > other.f_cost; }
    };
};