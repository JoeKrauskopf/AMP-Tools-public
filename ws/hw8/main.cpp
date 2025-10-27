// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"
#include "hw/HW8.h"
#include "MyMultiAgentPlanners.h"
#include <fstream>
#include <iomanip> // for std::setprecision

using namespace amp;

void timer_example() {
    double startTime;
    amp::Timer timer("timer");
    for (int i=0; i < 5; ++i) {
        startTime = timer.now(TimeUnit::ms);  
        std::cout << "Press any key to continue...\n";
        std::cin.get();
        std::cout << "Time since last run: " << timer.now(TimeUnit::ms) - startTime << std::endl;
    }
    timer.stop();
    std::cout << "Total time since last run: " << Profiler::getTotalProfile("timer") << std::endl;
}

// for saving benchmark data
void saveBenchmarkCSV(const std::string& filename, 
                      const std::vector<int>& numAgentsList,
                      const std::list<std::vector<double>>& compTimesAll,
                      const std::list<std::vector<double>>& treeSizesAll)
{
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "[ERROR] Could not open file " << filename << " for writing.\n";
        return;
    }

    // Header
    file << "numAgents,compTime,treeSize\n";

    auto compIt = compTimesAll.begin();
    auto treeIt = treeSizesAll.begin();
    for (size_t idx = 0; idx < numAgentsList.size(); ++idx, ++compIt, ++treeIt) {
        int numAgents = numAgentsList[idx];
        const std::vector<double>& compTimes = *compIt;
        const std::vector<double>& treeSizes = *treeIt;

        for (size_t i = 0; i < compTimes.size(); ++i) {
            file << numAgents << "," 
                 << std::fixed << std::setprecision(6) << compTimes[i] << "," 
                 << treeSizes[i] << "\n";
        }
    }

    file.close();
    std::cout << "[INFO] Saved benchmark CSV: " << filename << std::endl;
}

int main(int argc, char** argv) {
    // Initializing workspace 1 with 3 agents
    amp::RNG::seed(amp::RNG::randiUnbounded());
    MultiAgentPath2D path;
    MultiAgentProblem2D problem = HW8::getWorkspace1(2);
    std::vector<std::vector<Eigen::Vector2d>> collision_states;

    // Solve using a centralized approach
    MyCentralPlanner central_planner;
    path = central_planner.plan(problem);
    bool isValid = HW8::check(path, problem, collision_states);
    Visualizer::makeFigure(problem, path, collision_states);

    // Solve using a decentralized approach
    MyDecentralPlanner decentral_planner;
    collision_states = {{}};
    path = decentral_planner.plan(problem);
    isValid = HW8::check(path, problem, collision_states);
    //HW8::generateAndCheck(decentral_planner, path, problem, collision_states);
    Visualizer::makeFigure(problem, path, collision_states);


    // BENCHMARKING

    LOG("STARTING BENCHMARKING");

    bool coupleBench = 0;
    bool decoupleBench = 1;
    int numBenchmarks = 100;
    struct MultiAgentBenchmarkResult {
        std::string label;
        int numValidPaths = 0;
        std::vector<double> computationTimes;  // ms
        std::vector<int> treeSizes;            // number of nodes in RRT
    };

    std::list<std::vector<double>> compTimesCentralAll;
    std::list<std::vector<double>> treeSizesCentralAll;

    std::list<std::vector<double>> compTimesDecentralAll;
    std::list<std::vector<double>> treeSizesDecentralAll;

    std::vector<std::string> labelsAll;


    for (int num_agents = 2; num_agents <= 6; ++num_agents) {
        MultiAgentProblem2D problemBench = HW8::getWorkspace1(num_agents);

        std::vector<double> compTimesCentral;
        std::vector<int> treeSizesCentral;

        std::vector<double> compTimesDecentral;
        std::vector<int> treeSizesDecentral;

        if (coupleBench) {
            for (int i = 0; i < numBenchmarks; ++i) {
                MyCentralPlanner central_planner;
               
                amp::Timer timer("MultiRRT_Benchmark");

                amp::MultiAgentPath2D pathBench = central_planner.plan(problemBench);

                double elapsed_ms = timer.now(amp::TimeUnit::ms);
                
                
                
                if (elapsed_ms < 0.0) {
                    elapsed_ms = compTimesCentral.back();
                }
                

                compTimesCentral.push_back(elapsed_ms);
                treeSizesCentral.push_back(central_planner.getTreeSize());

                LOG("[Central][" << num_agents << " agents][Run " << i+1 
                    << "] Time: " << elapsed_ms << " ms | Tree size: " 
                    << central_planner.getTreeSize() 
                    << " | Valid path: " << (!pathBench.agent_paths.empty()));
            }

            // Boxplots for central planner
            std::list<std::vector<double>> compTimesData = {compTimesCentral};
            std::list<std::vector<double>> treeSizesData = {std::vector<double>(treeSizesCentral.begin(), treeSizesCentral.end())};
            std::vector<std::string> labels = {"Centralized RRT (" + std::to_string(num_agents) + " agents)"};

            Visualizer::makeBoxPlot(compTimesData, labels, "Computation Time (ms)", "Planner", "Time (ms)");
            Visualizer::makeBoxPlot(treeSizesData, labels, "Tree Size", "Planner", "Number of Nodes");
            compTimesCentralAll.push_back(compTimesCentral);
            treeSizesCentralAll.push_back(std::vector<double>(treeSizesCentral.begin(), treeSizesCentral.end()));
    
        }

        if (decoupleBench) {
            for (int i = 0; i < numBenchmarks; ++i) {
                MyDecentralPlanner decentral_planner;

                // Planner run (elapsed_ms returned only if successful)
                amp::MultiAgentPath2D pathBench = decentral_planner.plan(problemBench);
                double elapsed_ms = decentral_planner.getLastElapsedMs();
                // Record only successful run times
                compTimesDecentral.push_back(elapsed_ms);
                treeSizesDecentral.push_back(decentral_planner.getTreeSize());

                LOG("[Decentral][" << num_agents << " agents][Run " << i+1 
                    << "] Time: " << elapsed_ms << " ms | Tree size: " 
                    << decentral_planner.getTreeSize() 
                    << " | Valid path: " << (!pathBench.agent_paths.empty()));
            }

            // Boxplots
            std::list<std::vector<double>> compTimesData = {compTimesDecentral};
            std::list<std::vector<double>> treeSizesData = {std::vector<double>(treeSizesDecentral.begin(), treeSizesDecentral.end())};
            std::vector<std::string> labels = {"Decoupled RRT (" + std::to_string(num_agents) + " agents)"};

            LOG("FINISHED RUNS NOW PLOTTING");

            Visualizer::makeBoxPlot(compTimesData, labels, "Computation Time (ms)", "Planner", "Time (ms)");
            Visualizer::makeBoxPlot(treeSizesData, labels, "Tree Size", "Planner", "Number of Nodes");

            compTimesDecentralAll.push_back(compTimesDecentral);
            treeSizesDecentralAll.push_back(std::vector<double>(treeSizesDecentral.begin(), treeSizesDecentral.end()));
        }
        labelsAll.push_back(std::to_string(num_agents) + " agents");

    }
    
    // --- Combined Boxplots ---
    if (coupleBench) {
        Visualizer::makeBoxPlot(compTimesCentralAll, labelsAll, 
            "Centralized RRT: Computation Time", "Number of Agents", "Time (ms)");
        Visualizer::makeBoxPlot(treeSizesCentralAll, labelsAll, 
            "Centralized RRT: Tree Size", "Number of Agents", "Number of Nodes");
    }

    if (decoupleBench) {
        LOG("compTimesDecentralAll size: " << compTimesDecentralAll.size());
        LOG("treeSizesDecentralAll size: " << treeSizesDecentralAll.size());
        LOG("labelsAll size: " << labelsAll.size());
        Visualizer::makeBoxPlot(compTimesDecentralAll, labelsAll, 
            "Decoupled RRT: Computation Time", "Number of Agents", "Time (ms)");
        Visualizer::makeBoxPlot(treeSizesDecentralAll, labelsAll, 
            "Decoupled RRT: Tree Size", "Number of Agents", "Number of Nodes");
    }

    LOG("Finished plotting, saving data");

    // save data to CSV
    std::vector<int> numAgentsList = {2,3,4,5,6};

    // Save coupled planner results
    if(coupleBench) {
        saveBenchmarkCSV("coupled_benchmark.csv", numAgentsList, compTimesCentralAll, treeSizesCentralAll);
        std::cout << "Saved Coupled Benchmark Data Successfully!" << std::endl;
    }

    
    // Save decoupled planner results
    if(decoupleBench) {
        saveBenchmarkCSV("decoupled_benchmark.csv", numAgentsList, compTimesDecentralAll, treeSizesDecentralAll);
        std::cout << "Saved Decoupled Benchmark Data Successfully!" << std::endl;
    }




    // Visualize and grade methods
    Visualizer::saveFigures();
    bool grade = false;
    if(grade) {
        //MyCentralPlanner.max_iter = 10000;
        //MyDecentralPlanner.max_iter = 40000;

        HW8::grade<MyCentralPlanner, MyDecentralPlanner>("joseph.krauskopf@colorado.edu", argc, argv, std::make_tuple(), std::make_tuple());

    }
    return 0;
}