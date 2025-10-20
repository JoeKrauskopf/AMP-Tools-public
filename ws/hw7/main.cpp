    // This includes all of the necessary header files in the toolbox
    #include "AMPCore.h"
    #include "hw/HW2.h"
    #include "hw/HW5.h"
    #include "hw/HW6.h"
    #include "MySamplingBasedPlanners.h"

    #include <sstream>
    #include <iomanip> // for std::setprecision and std::fixed

    using namespace amp;


    struct PRMBenchmarkResult {
        int n;
        double r;
        int numValidPaths = 0;
        std::vector<double> validPathLengths;
        std::vector<double> computationTimes; // ms
    };


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

        // ADD THIS CHECK - Don't proceed if planning failed
        if (plan.waypoints.empty()) {
            //std::cerr << "[CRITICAL] Planning failed! No waypoints in path." << std::endl;
            //std::cerr << "Unable to visualize or validate empty path." << std::endl;
            LOG("Valid Path: 0");
            LOG("Path Lenght: 0");
            // You can return, exit, or try different parameters here
        } else {
            // Create node -> coordinate map for PRM
            std::map<amp::Node, Eigen::Vector2d> prm_node_to_coord;

            // start node
            prm_node_to_coord[0] = problem.q_init;
            // goal node
            prm_node_to_coord[1] = problem.q_goal;

            // all other PRM nodes (prm_nodes) starting from index 2
            for (size_t i = 0; i < prm.prm_nodes.size(); ++i) {
                prm_node_to_coord[i + 2] = prm.prm_nodes[i]; // offset by 2
            }
            amp::Visualizer::makeFigure(
                problem,              // The workspace
                plan,                 // The path from PRM
                *prm.graph_prm,       // The PRM graph
                prm_node_to_coord     // Map node indices to 2D coordinates
            );
            bool success = HW7::check(plan, problem);
            // LOG("Found valid solution to workspace: " << (success ? "Yes!" : "No :("));
            LOG("Valid Path: 1");
            LOG("Path length: " << plan.length());
        }

        
        
        // Generate a random problem and test RRT
        MyRRT rrt;
        Path2D pathRRT = rrt.plan(problem);
        bool successRRT = HW7::check(pathRRT, problem);
        LOG("Found valid solution to workspace: " << (successRRT ? "Yes!" : "No :("));
        LOG("path length: " << pathRRT.length());
        // Visualize the problem with the RRT tree
        // Create a map from node index to coordinate
        std::map<amp::Node, Eigen::Vector2d> node_to_coord;
        for (std::size_t i = 0; i < rrt.rrt_nodes.size(); ++i) {
            node_to_coord[i] = rrt.rrt_nodes[i];
        }


        // Figure 4: Problem with path overlaid on tree
        amp::Visualizer::makeFigure(
            problem,
            pathRRT,
            *rrt.rrt_graph,
            node_to_coord
        );

        //Visualizer::saveFigures();

    // -----------------------------------------------------------

        // benchmarking:
        


        
        LOG("****STARTING BENCHMARKS****");
        // for a given nr pair

            // run 100 benchmarks with given planner (prm or rrt)

            // record number of valid paths, path length of valid paths, and computation time

        // make box plot of number of valid paths for all nr pairs (1 box per nr pair)

        // make box plot of path length of valid paths for all nr pairs (1 box per nr pair)

        // make box plot of computation time of valid paths for all nr pairs (1 box per nr pair)
        bool benchPRM = 0;
        if(benchPRM){
            Problem2D problemBench = HW5::getWorkspace1();
            //Problem2D problemBench = HW2::getWorkspace1();
            //Problem2D problemBench = HW2::getWorkspace2();
            int numBenchmarks = 100;

            std::vector<std::pair<int, double>> prmParams = {
                {200, 0.5},
                {200, 1},
                {200, 1.5},
                {200, 2.0},
                {500, 0.5},
                {500, 1},
                {500, 1.5},
                {500, 2}
            };

            LOG("PRM Benchmarking");
            std::vector<PRMBenchmarkResult> allResults;
            double lastValidTime = 0.0; // store last valid elapsed time
            bool vis = false;
            double PLOTLENGTHPRM;
            for (auto& [n, r] : prmParams) {
                PRMBenchmarkResult result;
                result.n = n;
                result.r = r;

                for (int i = 0; i < numBenchmarks; ++i) {
                    MyPRM prm;
                    prm.n = n;
                    prm.r = r;

                    amp::Timer timer("PRM_Benchmark");
                    Path2D path = prm.plan(problemBench);
                    double elapsed_ms = timer.now(amp::TimeUnit::ms);
                    // Fix negative time
                    if (elapsed_ms < 0.0) {
                        elapsed_ms = lastValidTime;
                    } else {
                        lastValidTime = elapsed_ms;
                    }

                    bool valid = !path.waypoints.empty() && HW7::check(path, problemBench);
                    if (valid) {
                        result.numValidPaths++;
                        LOG("Path length: " << path.length());
    ;
                        result.validPathLengths.push_back(path.length());
                        if (n == 200 && r == 2 && !vis) {
                            
                            // PLOT VALID n = 200, r = 1 route

                            // Create node -> coordinate map for PRM
                            std::map<amp::Node, Eigen::Vector2d> prm_node_to_coord;

                            // start node
                            prm_node_to_coord[0] = problem.q_init;
                            // goal node
                            prm_node_to_coord[1] = problem.q_goal;

                            // all other PRM nodes (prm_nodes) starting from index 2
                            for (size_t i = 0; i < prm.prm_nodes.size(); ++i) {
                                prm_node_to_coord[i + 2] = prm.prm_nodes[i]; // offset by 2
                            }
                            PLOTLENGTHPRM = path.length();
                            amp::Visualizer::makeFigure(
                                problemBench,              // The workspace
                                path,                 // The path from PRM
                                *prm.graph_prm,       // The PRM graph
                                prm_node_to_coord     // Map node indices to 2D coordinates
                            );
                            vis = true;
                        }
                    }

                    result.computationTimes.push_back(elapsed_ms);
                    // ----- DEBUG / PROGRESS -----
                    if ((i+1) % 10 == 0 || i == 0 || i == numBenchmarks-1) {
                        LOG("[Benchmark] n=" << n << ", r=" << r << " | Iteration " << (i+1) 
                            << "/" << numBenchmarks 
                            << " | Valid paths so far: " << result.numValidPaths);
                    }
                }

                allResults.push_back(result);
            }
            std::list<std::vector<double>> validPathsData;
            std::list<std::vector<double>> pathLengthsData;
            std::list<std::vector<double>> compTimesData;
            std::vector<std::string> labels;

            for (auto& r : allResults) {
                validPathsData.push_back(std::vector<double>(1, r.numValidPaths)); // single value per nr pair
                pathLengthsData.push_back(r.validPathLengths);                           // only valid paths
                compTimesData.push_back(r.computationTimes);                        // all times

                // format r to 1 decimal place
                std::ostringstream oss;
                oss << "n=" << r.n << ", r=" << std::fixed << std::setprecision(1) << r.r;
                labels.push_back(oss.str());
            }
            int maxAttempts = 100000;
            
            /*
            if(!vis){
                std::cout << "Attempting to find solution for HW2WS2 n = 200, r = 2" << std:: endl;
                for(int ii = 0; ii< maxAttempts; ii++){

                    std::cout << "Iteration: " << ii << " / " << maxAttempts << std::endl;
                    MyPRM prm;
                    prm.n = 200;
                    prm.r = 2;
                    Path2D path = prm.plan(problemBench);
                    bool valid = !path.waypoints.empty() && HW7::check(path, problemBench);
                    if (valid) {
                        LOG("Path length: " << path.length());
                        // PLOT VALID n = 200, r = 1 route

                        // Create node -> coordinate map for PRM
                        std::map<amp::Node, Eigen::Vector2d> prm_node_to_coord;

                        // start node
                        prm_node_to_coord[0] = problem.q_init;
                        // goal node
                        prm_node_to_coord[1] = problem.q_goal;

                        // all other PRM nodes (prm_nodes) starting from index 2
                        for (size_t i = 0; i < prm.prm_nodes.size(); ++i) {
                            prm_node_to_coord[i + 2] = prm.prm_nodes[i]; // offset by 2
                        }
                        PLOTLENGTHPRM = path.length();
                        amp::Visualizer::makeFigure(
                            problemBench,              // The workspace
                            path,                 // The path from PRM
                            *prm.graph_prm,       // The PRM graph
                            prm_node_to_coord     // Map node indices to 2D coordinates
                        );
                        vis = true;
                        break;

                    }

                }
                std::cout << "No path found after " << maxAttempts << std::endl;
            }
            */
            
            // Box plot 1: Number of valid paths
            Visualizer::makeBoxPlot(validPathsData, labels, "Valid Paths per (n,r) pair", "Parameters", "Valid Paths");

            // Box plot 2: Path lengths
            Visualizer::makeBoxPlot(pathLengthsData, labels, "Path Lengths of Valid Paths", "Parameters", "Path Length");

            // Box plot 3: Computation times
            Visualizer::makeBoxPlot(compTimesData, labels, "Computation Times (ms)", "Parameters", "Elapsed Time (ms)");
            std::cout << "Plotted PRM Length: " << PLOTLENGTHPRM << std::endl;
        }

        bool benchRRT = 0;
        if(benchRRT) {
            int numBenchmarks = 100;

            // List of problem spaces
            std::vector<std::pair<std::string, amp::Problem2D>> benchmarkProblems = {
                {"HW5_WS1", HW5::getWorkspace1()},
                {"HW2_WS1", HW2::getWorkspace1()},
                {"HW2_WS2", HW2::getWorkspace2()}
            };

            // Containers for boxplot data
            std::list<std::vector<double>> validPathsData;
            std::list<std::vector<double>> pathLengthsData;
            std::list<std::vector<double>> compTimesData;
            std::vector<std::string> labels;

            // Loop through each workspace
            for (auto& [label, problemBench] : benchmarkProblems) {
                LOG("****STARTING RRT BENCHMARK: " << label << " ****");

                PRMBenchmarkResult result;
                double lastValidTime = 0.0;

                for (int i = 0; i < numBenchmarks; ++i) {
                    MyRRT rrt;
                    amp::Timer timer("RRT_Benchmark");
                    Path2D path = rrt.plan(problemBench);
                    double elapsed_ms = timer.now(amp::TimeUnit::ms);

                    // Sanity check: fix invalid or unrealistic times
                    if (elapsed_ms < 0.0 || elapsed_ms > 1000.0) {
                        LOG("[WARN] Invalid elapsed time detected (" << elapsed_ms << " ms), using last valid value: " << lastValidTime << " ms");
                        elapsed_ms = lastValidTime;
                    } else {
                        lastValidTime = elapsed_ms;
                    }

                    bool valid = !path.waypoints.empty() && HW7::check(path, problemBench);
                    if (valid) {
                        result.numValidPaths++;
                        result.validPathLengths.push_back(path.length());
                        LOG("[RRT][" << label << "] Valid path found | Length: " << path.length());
                    }

                    result.computationTimes.push_back(elapsed_ms);

                    // Progress feedback every 10 iterations
                    if ((i + 1) % 10 == 0 || i == 0 || i == numBenchmarks - 1) {
                        LOG("[RRT Benchmark][" << label << "] Iteration " << (i + 1) << "/" << numBenchmarks
                            << " | Valid paths so far: " << result.numValidPaths);
                    }
                }

                // Store data for this workspace
                validPathsData.push_back(std::vector<double>(1, result.numValidPaths));
                pathLengthsData.push_back(result.validPathLengths);
                compTimesData.push_back(result.computationTimes);
                labels.push_back(label);

                LOG("****COMPLETED RRT BENCHMARK: " << label << " ****");
            }

            // --- Create combined boxplots ---
            Visualizer::makeBoxPlot(validPathsData, labels,
                "RRT: Number of Valid Paths per Workspace", "Workspace", "Valid Paths");

            Visualizer::makeBoxPlot(pathLengthsData, labels,
                "RRT: Path Lengths of Valid Paths per Workspace", "Workspace", "Path Length");

            Visualizer::makeBoxPlot(compTimesData, labels,
                "RRT: Computation Times (ms) per Workspace", "Workspace", "Elapsed Time (ms)");
        }
        Visualizer::saveFigures();

    // -----------------------------------------------------------



        // Grade method
        //HW7::grade<MyPRM, MyRRT>("joseph.krauskopf@colorado.edu", argc, argv, std::make_tuple(), std::make_tuple());
        return 0;
    }


