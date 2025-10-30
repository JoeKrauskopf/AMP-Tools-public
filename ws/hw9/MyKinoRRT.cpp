#include "MyKinoRRT.h"
#include <vector>
#include <iostream>
#include "collisionCheck.h"
#include "MyAStar.h"

// maybe implement ode45?


void MySingleIntegrator::propagate(Eigen::VectorXd& state, Eigen::VectorXd& control, double dt) {
    state += dt * control;
};

void MyFirstOrderUnicycle::propagate(Eigen::VectorXd& state, Eigen::VectorXd& control, double dt) {
    // state: 3x1
    // control: 2x1

    // x_dot = u_sigma * r * cos(theta)
    // y_dot = u_sigma * r * cos(theta)
    // theta_dot = u_omega

    // 1. unpack inputs:
    double x = state(0);
    double y = state(1);
    double theta = state(2);

    double u_sigma = control(0);
    double u_omega = control(1);

    //double r = 0.1; // PLACEHOLDER CONSIDER MAKING PUBLIC VARIABLE

    // 2. compute derivatives

    double x_dot = u_sigma * r * cos(theta);
    double y_dot = u_sigma * r * sin(theta);
    double theta_dot = u_omega;

    // 3. integrate
    
    if(simple){
        // use simple integrator
        x += x_dot * dt;
        y += y_dot * dt;
        theta += theta_dot * dt;

        // re-store variables
        state(0) = x;
        state(1) = y;
        state(2) = theta;


    } else {
        // use runge kutta 45
        // code provided by chatGPT

        auto f = [&](const Eigen::VectorXd& s) -> Eigen::VectorXd {
            Eigen::VectorXd ds(3);
            double theta = s(2);
            double u_sigma = control(0);
            double u_omega = control(1);

            ds(0) = u_sigma * r * cos(theta); // x_dot
            ds(1) = u_sigma * r * sin(theta); // y_dot
            ds(2) = u_omega;                  // theta_dot
            return ds;
        };

        // RK4 integration
        Eigen::VectorXd k1 = f(state);
        Eigen::VectorXd k2 = f(state + 0.5 * dt * k1);
        Eigen::VectorXd k3 = f(state + 0.5 * dt * k2);
        Eigen::VectorXd k4 = f(state + dt * k3);

        state += (dt / 6.0) * (k1 + 2*k2 + 2*k3 + k4);
        state(2) = angleWrap::wrap(state(2));
    }
};

void MySecondOrderUnicycle::propagate(Eigen::VectorXd& state, Eigen::VectorXd& control, double dt) {
    // r: wheel radius
    // sigma: pedaling angular velocity
    // v = r*sigma
    // omega = rotational velocity

    // state: 5x1
    // control: 2x1

    // x_dot = u_sigma * r * cos(theta)
    // y_dot = u_sigma * r * cos(theta)
    // theta_dot = u_omega
    // sigma_dot = u1
    // omega_dot = u2

    // 1. unpack inputs:
    double x = state(0);
    double y = state(1);
    double theta = state(2); // keep wrapped -pi to pi
    double sigma = state(3);
    double omega = state(4);

    double u_1 = control(0);
    double u_2 = control(1);

    // 2. calculate derivates
    double x_dot = sigma * r * cos(theta);
    double y_dot = sigma * r * sin(theta);
    double theta_dot = omega;
    double sigma_dot = u_1;
    double omega_dot = u_2;

    if(simple){
        // simple integrator
        x += x_dot * dt;
        y += y_dot * dt;
        theta += theta_dot * dt;
        sigma += sigma_dot * dt;
        omega += omega_dot * dt;

        theta =  std::max(-M_PI, std::min(M_PI, theta));

        // re-store variables
        state(0) = x;
        state(1) = y;
        state(2) = theta;
        state(3) = sigma;
        state(4) = omega;
    } else {
        // use runge kutta 45

        auto f = [&](const Eigen::VectorXd& s) -> Eigen::VectorXd {
            Eigen::VectorXd ds(5);

            double x = s(0);
            double y = s(1);
            double theta = s(2);
            double sigma = s(3);
            double omega = s(4);

            double u1 = control(0);
            double u2 = control(1);

            ds(0) = sigma * r * cos(theta); // x_dot
            ds(1) = sigma * r * sin(theta); // y_dot
            ds(2) = omega;                  // theta_dot
            ds(3) = u1;                     // sigma_dot
            ds(4) = u2;                     // omega_dot
            return ds;
        };

        // RK4 integration
        Eigen::VectorXd k1 = f(state);
        Eigen::VectorXd k2 = f(state + 0.5 * dt * k1);
        Eigen::VectorXd k3 = f(state + 0.5 * dt * k2);
        Eigen::VectorXd k4 = f(state + dt * k3);

        state += (dt / 6.0) * (k1 + 2*k2 + 2*k3 + k4);
        //state(2) =  std::max(-M_PI, std::min(M_PI, state(2)));
        state(2) = angleWrap::wrap(state(2));

    }
};

void MySimpleCar::propagate(Eigen::VectorXd& state, Eigen::VectorXd& control, double dt) {
    // state: 5x1
    // control: 2x1
    double L = agent_dim.length;
    double W = agent_dim.width;

    //std::cout << "[PROPAGATE] Agent L: " << L << " Agent W: " << W << std::endl;

    // 1. unpack inputs
    double x = state(0);
    double y = state(1);
    double theta = state(2); // in radians
    double v = state(3);
    double phi = state(4); // wrapped from -pi/2 to pi/2
    
    double u1 = control(0);
    double u2 = control(1);

    // 2. compute derivatives
    double x_dot = v*cos(theta);
    double y_dot = v*sin(theta);
    double theta_dot = (v/L)*tan(phi);
    double v_dot = u1;
    double phi_dot = u2;

    if(simple){
        // simple integration
        x += x_dot*dt;
        y += y_dot*dt;
        theta += theta_dot*dt;
        v += v_dot*dt;
        phi += phi_dot*dt;
        phi = std::max(-M_PI_2, std::min(M_PI_2, phi)); // wrap phi

        state(0) = x;
        state(1) = y;
        state(2) = theta;
        state(3) = v;
        state(4) = phi;
    } else {
        // runge kutta 45
        auto f = [&](const Eigen::VectorXd& s) -> Eigen::VectorXd {
            Eigen::VectorXd ds(5);

            double x = s(0);
            double y = s(1);
            double theta = s(2);
            double v = s(3);
            double phi = s(4);

            double u1 = control(0);
            double u2 = control(1);

            ds(0) = v*cos(theta); // x_dot
            ds(1) = v*sin(theta); // y_dot
            ds(2) = (v/L)*tan(phi);                  // theta_dot
            ds(3) = u1;                     // sigma_dot
            ds(4) = u2;                     // omega_dot
            return ds;
        };

        // RK4 integration
        Eigen::VectorXd k1 = f(state);
        Eigen::VectorXd k2 = f(state + 0.5 * dt * k1);
        Eigen::VectorXd k3 = f(state + 0.5 * dt * k2);
        Eigen::VectorXd k4 = f(state + dt * k3);

        state += (dt / 6.0) * (k1 + 2*k2 + 2*k3 + k4);
       // state(4) = std::max(-M_PI_2, std::min(M_PI_2, state(4))); // wrap phi
        state(2) = angleWrap::wrap(state(2)); // wrap theta
        state(4) = angleWrap::wrap(state(4)); // wrap theta
    }

};




amp::KinoPath MyKinoRRT::plan(const amp::KinodynamicProblem2D& problem, amp::DynamicAgent& agent) {
    amp::KinoPath path;
    Eigen::VectorXd state = problem.q_init;
    collision coll;
    bool debug = true;

    // add debug statements about the agent type and problem info
    if(debug) {
        std::cout << "[DEBUG] Initial state (size=" << state.size() << "): " << state.transpose() << std::endl;

        std::cout << "[DEBUG] Number of state bounds: " << problem.q_bounds.size() << std::endl;
        for (size_t i = 0; i < problem.q_bounds.size(); i++) {
            auto [min_val, max_val] = problem.q_bounds[i];
            std::cout << "  State " << i << " bounds: [" << min_val << ", " << max_val << "]" << std::endl;
        }

        std::cout << "[DEBUG] Number of control bounds: " << problem.u_bounds.size() << std::endl;
        for (size_t i = 0; i < problem.u_bounds.size(); i++) {
            auto [min_val, max_val] = problem.u_bounds[i];
            std::cout << "  Control " << i << " bounds: [" << min_val << ", " << max_val << "]" << std::endl;
        }

        std::cout << "[DEBUG] Time step bounds: [" << problem.dt_bounds.first << ", " << problem.dt_bounds.second << "]" << std::endl;
        std::cout << "[DEBUG] Agent type: " << typeid(agent).name() << std::endl;
        std::cout << "[DEBUG] Is point agent: " << problem.isPointAgent << std::endl;

        // New: goal bounds
        std::cout << "[DEBUG] Number of goal bounds: " << problem.q_goal.size() << std::endl;
        for (size_t i = 0; i < problem.q_goal.size(); i++) {
            auto [min_goal, max_goal] = problem.q_goal[i];
            std::cout << "  Goal " << i << " bounds: [" << min_goal << ", " << max_goal << "]" << std::endl;
        }

        // New: goal state (center)
        Eigen::VectorXd goal_center(problem.q_goal.size());
        for (size_t i = 0; i < problem.q_goal.size(); i++) {
            auto [min_goal, max_goal] = problem.q_goal[i];
            goal_center(i) = 0.5 * (min_goal + max_goal);
        }
        std::cout << "[DEBUG] Goal center: " << goal_center.transpose() << std::endl;
    }
    



    // NOW NEED TO BUILD KINODYNAMIC RRT
// ***********************************************************************************************
    // helper functions:

    // sampleState():
        // random values for state components
    auto sampleState = [&]() -> Eigen::VectorXd {
        // randomize value for every state within state bounds
        Eigen::VectorXd x(problem.q_bounds.size());
            
        for (int i = 0; i < problem.q_bounds.size(); i++) {
            auto [min_val, max_val] = problem.q_bounds[i];
            x(i) = min_val + ((double)rand() / RAND_MAX) * (max_val - min_val);
        }
        return x;
        
    };

    // sample random control input within control bounds
    auto sampleControl = [&]() -> Eigen::VectorXd {
        Eigen::VectorXd u(problem.u_bounds.size());
        for (int i = 0; i < problem.u_bounds.size(); i++){
            auto [min_val, max_val] = problem.u_bounds[i];
            u(i) = min_val + ((double)rand() / RAND_MAX) * (max_val - min_val);
        }
        return u;
    };

    // sample random time step within time step bounds
    auto sampleDT = [&]() -> double {
        auto [dt_min, dt_max] = problem.dt_bounds;
        return dt_min + ((double)rand() / RAND_MAX) * (dt_max - dt_min);
    };

    auto inGoal = [&](const Eigen::VectorXd& x) -> bool {
        // check each state of x to see if its in the goal bounds
        for (int i = 0; i < x.size(); i++) {
            auto [min_goal, max_goal] = problem.q_goal[i];
            if (x(i) < min_goal || x(i) > max_goal)
                return false;
        }
        return true;
    };

    // rho(x_a, x_b): 
        // distance metric between states
    auto rho = [&](const Eigen::VectorXd& x_a, const Eigen::VectorXd& x_b) -> double {
        double dist;

        // L2 norm of vectors x_a and x_b
        dist = (x_a - x_b).norm();

        return dist;
    };

    auto nearestIndex = [&](const std::vector<Eigen::VectorXd>& nodes, const Eigen::VectorXd& x_rand) -> int {
        int nearest = 0;
        double best_dist = std::numeric_limits<double>::infinity();
        for(int i = 0; i<(int)nodes.size(); i++){
            double d = rho(nodes[i], x_rand);
            if(d<best_dist) {
                best_dist = d;
                nearest = i;
            }
        }
        return nearest;
    };

    // collision checker
    auto inCollision = [&](const Eigen::VectorXd& x) -> bool {
        Eigen::Vector2d q(x(0), x(1)); // assume the first two elements of any state are x and y pos

        if(problem.isPointAgent) {
            // can use our basic workspace checker

            collision::Result res = coll.collisionCheckAllEnv(q,static_cast<const amp::Environment2D&>(problem));
            return res.hit;
        } else{
            // save for future implementation
            double theta = x(2); // assume 3rd element of every state is theta
            // implement this
            collision::Result res = coll.collisionCheckDynamicAgentAllEnv(q,agent,theta,static_cast<const amp::Environment2D&>(problem));
            
            return res.hit;
        }
    };

    auto isStateValid = [&](const Eigen::VectorXd& x) -> bool {

        if(problem.isPointAgent) {
            for (int i = 0; i < problem.q_bounds.size(); i++) {
                auto [min_val, max_val] = problem.q_bounds[i];
                if(x(i) < min_val || x(i) > max_val){
                    return false;
                }
            }
            return true;
        } else {
            // check states first
            for (int i = 0; i < problem.q_bounds.size(); i++) {
                auto [min_val, max_val] = problem.q_bounds[i];
                if(x(i) < min_val || x(i) > max_val){
                    return false;
                }
            }
            // check all four corners of the robot
            double L = agent.agent_dim.length;
            double W = agent.agent_dim.width;
            double theta = x(2);
            Eigen::Vector2d q(x(0), x(1)); // robot center

            // Local corners
            std::vector<Eigen::Vector2d> localCorners = {
                Eigen::Vector2d(0, -W/2),  // back-left
                Eigen::Vector2d(L, -W/2),  // front-left
                Eigen::Vector2d(L, W/2),   // front-right
                Eigen::Vector2d(0, W/2)    // back-right
            };

            // Rotation matrix
            double cosTheta = std::cos(theta);
            double sinTheta = std::sin(theta);
            Eigen::Matrix2d R;
            R << cosTheta, -sinTheta,
                sinTheta, cosTheta;

            // Transform to world coordinates
            for (const auto& corner : localCorners) {
                Eigen::Vector2d worldCorner = R * corner + q;
                if (worldCorner(0) < problem.q_bounds[0].first || worldCorner(0) > problem.q_bounds[0].second)
                    return false;
                if (worldCorner(1) < problem.q_bounds[1].first || worldCorner(1) > problem.q_bounds[1].second)
                    return false;
            }

            return true;
        }
    };

    
    
    // generateLocalTrajectory(x_a, x_b)
        // 2 point boundary problem
        // approach 1: random sampling
        // approach 2: find best-out-of-many random controls
    auto generateLocalTrajectory = [&](const Eigen::VectorXd& x_near, const Eigen::VectorXd& x_target) -> TrajectoryStep {
        const int numControls = max_controls;
        TrajectoryStep best_step;
        double best_dist = std::numeric_limits<double>::infinity();
        int valid_controls = 0;
        int collision_controls = 0;

        for(int i = 0; i<numControls; i++){
            Eigen::VectorXd u = sampleControl();
            double dt = sampleDT();
            Eigen::VectorXd x_new = x_near;
            const int n_substeps = 1; // remove redundant edge checking
            double sub_dt = dt / n_substeps;
            bool collision_free = true;

            for(int k = 0; k< n_substeps; k++){
                agent.propagate(x_new, u, sub_dt); // forward propagate 1 timestep
                if(inCollision(x_new)) {
                    collision_free = false;
                    collision_controls++;
                    break;
                }
            }

            if(collision_free){
                valid_controls++;
                double dist = rho(x_new, x_target);
                if (dist < best_dist) {
                    best_dist = dist;
                    best_step.state = x_new;
                    best_step.control = u;
                    best_step.dt = dt;
                }
            }
        }
        if(valid_controls == 0) {
            //std::cout << "[DEBUG] generateLocalTrajectory: ALL " << numControls 
                    //<< " controls caused collision!" << std::endl;
        
        }
        return best_step;
    };

    auto edge_collision_free = [&](const Eigen::VectorXd& x_near, const TrajectoryStep& x_new) -> bool {
        const int n_substeps = 500;
        double sub_dt = x_new.dt / n_substeps;
        Eigen::VectorXd x_temp = x_near;
        Eigen::VectorXd control = x_new.control;

        for (int k = 0; k < n_substeps; k++) {
            agent.propagate(x_temp, control, sub_dt);
            if (inCollision(x_temp) || !isStateValid(x_temp)) return false;
        }
        return true;
    };

    // isSubTrajectoryValid(lambda,0,steps)
        // incremental approach to check if intermediate states are valid

// *********************************************************************************************

    // initial condition setup 

    Eigen::VectorXd x0 = problem.q_init;
    Eigen::VectorXd x_goal(problem.q_goal.size());
    for (size_t i = 0; i < problem.q_goal.size(); i++) {
        auto [min_goal, max_goal] = problem.q_goal[i];
        x_goal(i) = 0.5 * (min_goal + max_goal);
    }

    // RIGHT AFTER initial condition setup in plan():
    std::cout << "[DEBUG] ===== COLLISION DIAGNOSIS =====" << std::endl;
    std::cout << "[DEBUG] Initial state: " << x0.transpose() << std::endl;
    std::cout << "[DEBUG] Initial state in collision: " << inCollision(x0) << std::endl;

    const int n_state = x0.size(); // number of states
    int goal_idx = -1;
    bool found = false;

    // T <- tree rooted at x0
    // Store states separately
    std::vector<Eigen::VectorXd> rrtStates;
    auto T = std::make_shared<amp::Graph<double>>();

    // Store trajectory info separately: child_idx -> edge info
    std::unordered_map<int, RRTEdge> edgeMap;

    // Root node
    int root_id = 0;
    rrtStates.push_back(problem.q_init);

    std::cout << "[DEBUG] Starting KinoRRT" << std::endl;

    // **MAIN LOOP**
    for(int iter = 0; iter < max_iterations; iter++){
        //std::cout << "Iteration: " << iter << " Percent complete: " << (static_cast<double>(iter) / max_iterations) * 100.0 << "%" << std::endl;
        if(iter % std::max(1, max_iterations / 100) == 0) {
            std::cout << "[DEBUG] Percent Complete: " << (static_cast<double>(iter) / max_iterations) * 100.0  << "%" << std::endl;
        }

        // generate random sample
        Eigen::VectorXd x_rand;
        if ((double)rand()/RAND_MAX < goalBias) {
            x_rand = Eigen::VectorXd(problem.q_goal.size());
            for (size_t i = 0; i < problem.q_goal.size(); i++) {
                auto [min_goal, max_goal] = problem.q_goal[i];
                x_rand(i) = min_goal + ((double)rand() / RAND_MAX) * (max_goal - min_goal);
            }
        } else {
            x_rand = sampleState();
        }
        //std::cout << "[DEBUG] Random Sample Generated" << std::endl;

        // find nearest node
        int nearest_id = nearestIndex(rrtStates, x_rand);
        Eigen::VectorXd x_near = rrtStates[nearest_id];

        //std::cout << "[DEBUG] Found x_near" << std::endl;

        // generate local trajectory
        TrajectoryStep step = generateLocalTrajectory(x_near, x_rand); // x_new

        
        if(step.state.size() > 0) {
            //std::cout << "[DEBUG] Generated Valid Trajectory step" << std::endl;

            if(edge_collision_free(x_near, step)) {
                //std::cout << "[DEBUG] edge is collision free" << std::endl;
                int new_id = (int)rrtStates.size();
                rrtStates.push_back(step.state);
                //std::cout << "[DEBUG] Added state to rrtStates" << std::endl;

                double w = step.control.norm() + step.dt;
                T->connect((uint32_t)nearest_id, (uint32_t)new_id, w);
                T->connect((uint32_t)new_id, (uint32_t)nearest_id, w);
                //std::cout << "[DEBUG] connected node to graph" << std::endl;

                edgeMap[new_id] = RRTEdge{step.state, step.control, step.dt};
                //std::cout << "[DEBUG] added control and dt to edgeMap" << std::endl;

                if(inGoal(step.state)) {
                    std::cout << "[DEBUG] Goal Found!" << std::endl;
                    std::cout << "[DEBUG] step.state: " << step.state.transpose() << std::endl;
                    goal_idx = new_id;  // Just use step.state
                    /*
                    // Try to reach exact goal
                    Eigen::VectorXd delta = x_goal - step.state;
                    Eigen::VectorXd u_snap = delta / step.dt;
                    Eigen::VectorXd test_state = step.state;
                    agent.propagate(test_state, u_snap, step.dt);
                    
                    if(inGoal(test_state)) {
                        goal_idx = (int)rrtStates.size();
                        rrtStates.push_back(test_state);  // Use propagated state, not x_goal
                        edgeMap[goal_idx] = RRTEdge{u_snap, step.dt};
                        T->connect(new_id, goal_idx, delta.norm());
                    } else {
                        goal_idx = new_id;  // Just use step.state
                    }
                    */

                    found = true;
                    break;
                }
            }
        }
        
        
    }


    Eigen::VectorXd goal_center(problem.q_goal.size());
    for (size_t i = 0; i < problem.q_goal.size(); i++) {
        auto [min_goal, max_goal] = problem.q_goal[i];
        goal_center(i) = 0.5 * (min_goal + max_goal); // midpoint
    }
    int nearest_goal_idx = found ? goal_idx : nearestIndex(rrtStates, goal_center);
        

    // then setup and run A*
    amp::ShortestPathProblem rrt_problem;

    rrt_problem.graph = T;
    rrt_problem.init_node = 0;         // root node
    rrt_problem.goal_node = nearest_goal_idx;  // either goal or nearest to goal
    amp::SearchHeuristic zeroHeuristic;

    std::unordered_map<amp::Node, Eigen::VectorXd> nodeStates;
    for (size_t i = 0; i < rrtStates.size(); i++) {
        nodeStates[i] = rrtStates[i];
    }

    MyAStarAlgo algo;
    MyAStarAlgo::GraphSearchResult Astar_result = algo.search(rrt_problem, zeroHeuristic);
    // backout  waypoints and controls and durations
    if (Astar_result.success) {
        // Iterate through the list using iterators
        for (auto it = Astar_result.node_path.begin(); it != Astar_result.node_path.end(); ++it) {
            int path_idx = *it;  // Dereference the iterator to get the node index
            
            // Skip the root node (it has no incoming edge)
            if (path_idx == 0) {
                path.waypoints.push_back(rrtStates[path_idx]);
                continue;
            }
            
            // For all other nodes, add state, control, and duration from edgeMap
            path.waypoints.push_back(edgeMap[path_idx].state);
            path.controls.push_back(edgeMap[path_idx].control);
            path.durations.push_back(edgeMap[path_idx].duration);
        }
        
        path.valid = true;
    } else {
        std::cerr << "RRT: no path found!" << std::endl;
    }
    std::cout << "Last path.waypoint: " << path.waypoints.back().transpose() << std::endl;

    return path;
}
