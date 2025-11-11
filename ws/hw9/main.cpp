// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"
#include "hw/HW9.h"
#include "hw/HW2.h"
#include "MyKinoRRT.h"

using namespace amp;

// Load problems and map agent for quick testing
std::vector<KinodynamicProblem2D> problems = {HW9::getStateIntProblemWS1(), HW9::getStateIntProblemWS2(), HW9::getFOUniProblemWS1(), HW9::getFOUniProblemWS2(), HW9::getSOUniProblemWS1(), HW9::getSOUniProblemWS2(), HW9::getCarProblemWS1(), HW9::getParkingProblem()};
std::unordered_map<AgentType, std::function<std::shared_ptr<amp::DynamicAgent>()>> agentFactory = {
    {AgentType::SingleIntegrator, []() { return std::make_shared<MySingleIntegrator>(); }},
    {AgentType::FirstOrderUnicycle, []() { return std::make_shared<MyFirstOrderUnicycle>(); }},
    {AgentType::SecondOrderUnicycle, []() { return std::make_shared<MySecondOrderUnicycle>(); }},
    {AgentType::SimpleCar, []() { return std::make_shared<MySimpleCar>(); }}
};

void plotCarControls(const amp::KinoPath& path, const std::string& problem_name = "car") {
    if (path.controls.empty() || path.durations.empty()) {
        std::cerr << "Error: No control data to plot" << std::endl;
        return;
    }
    
    // Compute cumulative time
    std::vector<double> time_points;
    double cumulative_time = 0.0;
    time_points.push_back(cumulative_time);
    
    for (size_t i = 0; i < path.durations.size(); i++) {
        cumulative_time += path.durations[i];
        time_points.push_back(cumulative_time);
    }
    
    // Extract velocity (u1) and steering angle (u2) controls
    std::vector<double> velocities;
    std::vector<double> steering_angles;
    
    for (const auto& control : path.controls) {
        velocities.push_back(control(0));        // u1: velocity control
        steering_angles.push_back(control(1));   // u2: steering angle control
    }
    
    // Write to CSV for plotting
    std::string filename = problem_name + "_controls.csv";
    std::ofstream file(filename);
    file << "time,velocity,steering_angle\n";
    
    for (size_t i = 0; i < path.controls.size(); i++) {
        file << time_points[i] << "," << velocities[i] << "," << steering_angles[i] << "\n";
        // Also write the end point of this control segment (for step plot)
        file << time_points[i+1] << "," << velocities[i] << "," << steering_angles[i] << "\n";
    }
    file.close();
    
    std::cout << "Control data written to " << filename << std::endl;
    std::cout << "Total duration: " << cumulative_time << " seconds" << std::endl;
    std::cout << "Number of control segments: " << path.controls.size() << std::endl;
    
    // Print statistics
    double avg_vel = 0.0, avg_steer = 0.0;
    for (size_t i = 0; i < velocities.size(); i++) {
        avg_vel += velocities[i];
        avg_steer += steering_angles[i];
    }
    avg_vel /= velocities.size();
    avg_steer /= steering_angles.size();
    
    std::cout << "Average velocity control: " << avg_vel << std::endl;
    std::cout << "Average steering control: " << avg_steer << std::endl;
}

int main(int argc, char** argv) {
    // Select problem, plan, check, and visualize
    int select = 7;
    bool movie = true;
    KinodynamicProblem2D prob = problems[select];
    MyKinoRRT kino_planner;
    amp::Timer timer("Timer");
    KinoPath path = kino_planner.plan(prob, *agentFactory[prob.agent_type]());
    double time = timer.now(amp::TimeUnit::ms);
    LOG("Elapsed Time: " << time);
    

    HW9::check(path, prob);
    double totalLength = 0.0;
    if (path.valid)
        
     
        for (size_t i = 1; i < path.waypoints.size(); ++i) {
            totalLength += (path.waypoints[i] - path.waypoints[i-1]).norm();
        }
        
        LOG("Total Length: " << totalLength);
        Visualizer::makeFigure(prob, path, false); // Set to 'true' to render animation

        // plot control inputs vs time
        // Plot control inputs vs time
        if (prob.agent_type == AgentType::SimpleCar) {
            std::string problem_name = (select == 6) ? "car_ws1" : "parking";
            plotCarControls(path, problem_name);
        }
        
        if(movie)
            Visualizer::makeFigure(prob, path, true);

    // going to have to implement benchmarking
    // BENCHMARKING
    struct KinoBenchmarkResult {
        int n;
        int numUSamples;
        std::string label;
        std::vector<double> times;
        std::vector<double> pathLengths;
        int numValidPaths = 0;
    };

    std::vector<std::pair<int,int>> configs = {
        //{50000, 1},
        {50000, 5},
        //{50000, 10},
        //{50000, 15}
    };
    std::vector<int> problemSelects = {
        0,2,4
    };

    bool uni_bench = 0;
    bool car_bench = 1;
    int numUniBenchmarks  = 50;
    int numCarBenchmarks = 100;

    bool isValid;
    

    if (uni_bench) {
        for (int idx = 0; idx < problemSelects.size(); ++idx) {
            KinodynamicProblem2D probBench = problems[problemSelects[idx]];
            std::shared_ptr<amp::DynamicAgent> agent = agentFactory[probBench.agent_type]();

            std::string agentName;
            switch (probBench.agent_type) {
                case AgentType::SingleIntegrator: agentName = "SingleIntegrator"; break;
                case AgentType::FirstOrderUnicycle: agentName = "FirstOrderUnicycle"; break;
                case AgentType::SecondOrderUnicycle: agentName = "SecondOrderUnicycle"; break;
                default: agentName = "UnknownAgent"; break;
            }

            LOG("=== Benchmarking Agent: " << agentName << " ===");

            std::list<std::vector<double>> timeDataAll;
            std::list<std::vector<double>> lengthDataAll;
            std::list<std::vector<double>> validDataAll;
            std::vector<std::string> labelsAll;

            for (auto [n, usamples] : configs) {
                LOG("Running (n=" << n << ", |u_samples|=" << usamples << ") ...");

                KinoBenchmarkResult result;
                result.label = "|u|=" + std::to_string(usamples);

                for (int run = 0; run < numUniBenchmarks; ++run) {
                    MyKinoRRT kino;
                    kino.max_iterations = n;
                    kino.max_controls = usamples;

                    amp::Timer timer("BenchmarkTimer");
                    KinoPath path = kino.plan(probBench, *agent);
                    isValid = HW9::check(path, probBench);
                    double elapsed_ms = timer.now(amp::TimeUnit::ms);

                    result.times.push_back(elapsed_ms);
                    double totalLength = 0.0;
                    if (isValid) {
                        result.numValidPaths++;
                        
                        for (size_t i = 1; i < path.waypoints.size(); ++i)
                            totalLength += (path.waypoints[i] - path.waypoints[i-1]).norm();
                        result.pathLengths.push_back(totalLength);
                    } else {
                        //result.pathLengths.push_back(NAN); dont push back failed path lenghts
                    }

                    LOG("[" << agentName << "][Run " << run+1 << "] Time=" << elapsed_ms 
                        << "ms | Valid=" << isValid << " | Length: " << totalLength);
                }

                // Add data for plotting
                timeDataAll.push_back(result.times);
                lengthDataAll.push_back(result.pathLengths);
                validDataAll.push_back({ static_cast<double>(result.numValidPaths) }); // wrap in vector                labelsAll.push_back(result.label);
                labelsAll.push_back(result.label);

                LOG(">>> " << agentName << " |u|=" << usamples 
                    << " | Valid Paths: " << result.numValidPaths << "/" << numUniBenchmarks);
            }

            // ------------------------------------------------------------
            // Boxplots for this agent
            Visualizer::makeBoxPlot(timeDataAll, labelsAll, 
                "Computation Time - " + agentName, "|u_samples|", "Time (ms)");
            Visualizer::makeBoxPlot(lengthDataAll, labelsAll, 
                "Path Length - " + agentName, "|u_samples|", "Path Length");
            Visualizer::makeBoxPlot(validDataAll, labelsAll, 
                "Valid Paths - " + agentName, "|u_samples|", "Number of Valid Paths");
        }

        LOG("===== BENCHMARKING COMPLETE =====");
    }

    if (car_bench) {
        KinodynamicProblem2D probBench = problems[7];
        std::string agentName;

        std::shared_ptr<amp::DynamicAgent> agent = agentFactory[probBench.agent_type]();
        switch (probBench.agent_type) {
                case AgentType::SingleIntegrator: agentName = "SingleIntegrator"; break;
                case AgentType::FirstOrderUnicycle: agentName = "FirstOrderUnicycle"; break;
                case AgentType::SecondOrderUnicycle: agentName = "SecondOrderUnicycle"; break;
                case AgentType::SimpleCar: agentName = "SimpleCar"; break;
                default: agentName = "UnknownAgent"; break;
            }
        LOG("=== Benchmarking Agent: " << agentName << " ===");

        std::list<std::vector<double>> timeDataAll;
        std::list<std::vector<double>> lengthDataAll;
        std::list<std::vector<double>> validDataAll;
        std::vector<std::string> labelsAll;

        for (auto [n, usamples] : configs) {
            LOG("Running (n=" << n << ", |u_samples|=" << usamples << ") ...");

            KinoBenchmarkResult result;
            result.label = "|u|=" + std::to_string(usamples);

            for (int run = 0; run < numCarBenchmarks; ++run) {
                MyKinoRRT kino;
                kino.max_iterations = n;
                kino.max_controls = usamples;

                amp::Timer timer("BenchmarkTimer");
                KinoPath path = kino.plan(probBench, *agent);
                isValid = HW9::check(path, probBench);
                double elapsed_ms = timer.now(amp::TimeUnit::ms);

                result.times.push_back(elapsed_ms);
                double totalLength = 0.0;
                if (isValid) {
                    result.numValidPaths++;
                    
                    for (size_t i = 1; i < path.waypoints.size(); ++i)
                        totalLength += (path.waypoints[i] - path.waypoints[i-1]).norm();
                    result.pathLengths.push_back(totalLength);
                } else {
                    //result.pathLengths.push_back(NAN); dont push back failed paths
                }

                LOG("[" << agentName << "][Run " << run+1 << "] Time=" << elapsed_ms 
                    << "ms | Valid=" << isValid << " | Length: " << totalLength);
            }

            // Add data for plotting
            timeDataAll.push_back(result.times);
            lengthDataAll.push_back(result.pathLengths);
            validDataAll.push_back({ static_cast<double>(result.numValidPaths) }); // wrap in vector                labelsAll.push_back(result.label);
            labelsAll.push_back(result.label);

            LOG(">>> " << agentName << " |u|=" << usamples 
                << " | Valid Paths: " << result.numValidPaths << "/" << numUniBenchmarks);
        }

        // ------------------------------------------------------------
        // Boxplots for this agent
        Visualizer::makeBoxPlot(timeDataAll, labelsAll, 
            "Computation Time - " + agentName, "|u_samples|", "Time (ms)");
        Visualizer::makeBoxPlot(lengthDataAll, labelsAll, 
            "Path Length - " + agentName, "|u_samples|", "Path Length");
        Visualizer::makeBoxPlot(validDataAll, labelsAll, 
            "Valid Paths - " + agentName, "|u_samples|", "Number of Valid Paths");
            LOG("===== BENCHMARKING COMPLETE =====");
    }



    Visualizer::saveFigures();
    //HW9::grade<MyKinoRRT, MySingleIntegrator, MyFirstOrderUnicycle, MySecondOrderUnicycle, MySimpleCar>("joseph.krauskopf@colorado.edu", argc, argv, std::make_tuple(), std::make_tuple(), std::make_tuple(), std::make_tuple(), std::make_tuple());
    return 0;
}