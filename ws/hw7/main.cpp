// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"
#include "hw/HW2.h"
#include "hw/HW5.h"
#include "hw/HW6.h"
#include "MySamplingBasedPlanners.h"

using namespace amp;

int main(int argc, char** argv) {
    //HW7::hint(); // Consider implementing an N-dimensional planner 
    
    // Example of creating a graph and adding nodes for visualization
    std::shared_ptr<amp::Graph<double>> graphPtr = std::make_shared<amp::Graph<double>>();
    /*
    std::map<amp::Node, Eigen::Vector2d> nodes;
    
    std::vector<Eigen::Vector2d> points = {{3, 3}, {4, 5}, {5, 3}, {6, 5}, {5, 7}, {7, 3}}; // Points to add to the graph
    for (amp::Node i = 0; i < points.size(); ++i) nodes[i] = points[i]; // Add point-index pair to the map
    std::vector<std::tuple<amp::Node, amp::Node, double>> edges = {{0, 4, 1}, {0, 5, 1}, {4, 5, 1}, {1, 2, 1}, {1, 3, 1}, {2, 3, 1}}; // Edges to connect
    for (const auto& [from, to, weight] : edges) graphPtr->connect(from, to, weight); // Connect the edges in the graph
    graphPtr->print();
    */

    // Test PRM on Workspace1 of HW2
    //Problem2D problem = HW2::getWorkspace1();
    Problem2D problem = HW2::getWorkspace2();
    //Problem2D problem = HW5::getWorkspace1();
    //Problem2D problem = HW6::getWorksapce1();

    MyPRM prm;
    Path2D plan = prm.plan(problem);

    std::cout << "PRM path waypoints: " << plan.waypoints.size() << std::endl;

    // Check your path to make sure that it does not collide with the environment 
    bool success = HW7::check(plan, problem);

    LOG("Found valid solution to workspace 1: " << (success ? "Yes!" : "No :("));
    LOG("path length: " << plan.length());

    // Visualize PRM path only (no graph, no nodes)
    // TODO: figure out how to plot graph as well cause it looks cool
    Visualizer::makeFigure(problem, plan);

    // Generate a random problem and test RRT
    MyRRT rrt;
    Path2D pathRRT = rrt.plan(problem);
    bool successRRT = HW7::check(pathRRT, problem);
    LOG("Found valid solution to workspace 1: " << (successRRT ? "Yes!" : "No :("));
    LOG("path length: " << pathRRT.length());
    //HW7::generateAndCheck(rrt, path, problem);
    Visualizer::makeFigure(problem, pathRRT);
    Visualizer::saveFigures();

    // Grade method
    HW7::grade<MyPRM, MyRRT>("joseph.krauskopf@colorado.edu", argc, argv, std::make_tuple(), std::make_tuple());
    return 0;
}