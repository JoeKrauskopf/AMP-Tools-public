#include "MyCSConstructors.h"
#include "collisionCheck.h"
#include "AMPCore.h"
#include "hw/HW4.h"
#include "tools/Visualizer.h"
#include "ManipulatorSkeleton.h"
#include <queue>
#include <iostream>

////////////////////// THIS IS FROM HW4 //////////////////////

/* You can just move these classes to shared folder and include them instead of copying them to hw6 project*/

std::pair<std::size_t, std::size_t> MyGridCSpace2D::getCellFromPoint(double x0, double x1) const {
    // Implment your discretization procedure here, such that the point (x0, x1) lies within the returned cell
    // Get bounds from ConfigurationSpace2D
    auto x0_bounds = this->x0Bounds(); // returns std::pair<double, double>
    auto x1_bounds = this->x1Bounds(); // returns std::pair<double, double>
    
    double x0_min = x0_bounds.first;
    double x0_max = x0_bounds.second;
    double x1_min = x1_bounds.first;
    double x1_max = x1_bounds.second;
    
    // Get grid size from DenseArray2D
    auto grid_size = this->size(); // returns std::pair<std::size_t, std::size_t>
    std::size_t x0_cells = grid_size.first;
    std::size_t x1_cells = grid_size.second;

    double u = (x0 - x0_min) / (x0_max  - x0_min);
    double v = (x1 - x1_min) / (x1_max  - x1_min);

    
    // Scale into indices
    std::size_t cell_x = static_cast<std::size_t>(std::floor(u * x0_cells));
    std::size_t cell_y = static_cast<std::size_t>(std::floor(v * x1_cells));

    // Clamp to grid
    cell_x = std::min(cell_x, x0_cells - 1);
    cell_y = std::min(cell_y, x1_cells - 1);
    return {cell_x, cell_y};
}

// FIGURE OUT WHY THIS DOESNT WORK

// Override this method for computing all of the boolean collision values for each cell in the cspace
std::unique_ptr<amp::GridCSpace2D> MyManipulatorCSConstructor::construct(const amp::LinkManipulator2D& manipulator, const amp::Environment2D& env) {
    // Create an object of my custom cspace type (e.g. MyGridCSpace2D) and store it in a unique pointer. 
    // Pass the constructor parameters to std::make_unique()
    std::unique_ptr<MyGridCSpace2D> cspace_ptr = std::make_unique<MyGridCSpace2D>(m_cells_per_dim, m_cells_per_dim, env.x_min, env.x_max, env.y_min, env.y_max);
    // In order to use the pointer as a regular GridCSpace2D object, we can just create a reference
    MyGridCSpace2D& cspace = *cspace_ptr;
    std::cout << "Constructing C-Space for link manipulator" << std::endl;

    //std::cout << "Environment contains " << env.obstacles.size() << " obstacles:" << std::endl;

    /*
    for (size_t obs_idx = 0; obs_idx < env.obstacles.size(); ++obs_idx) {
        const auto& obstacle = env.obstacles[obs_idx];
        std::cout << "Obstacle " << obs_idx << " vertices:" << std::endl;
        for (size_t vert_idx = 0; vert_idx < obstacle.verticesCCW().size(); ++vert_idx) {
            const auto& vertex = obstacle.verticesCCW()[vert_idx];
            std::cout << "  Vertex " << vert_idx << ": (" << vertex.x() << ", " << vertex.y() << ")" << std::endl;
        }
    }
    */
    

    //std::cout <<"Num Links: " << manipulator.nLinks() << std::endl;

    // Print link lengths
    const std::vector<double>& linkLengths = manipulator.getLinkLengths(); //get link lengths;
    
    /*
    std::cout << "Link lengths: ";
    for (size_t i = 0; i < linkLengths.size(); ++i) {
        std::cout << linkLengths[i] << " ";
    }
    std::cout << std::endl;
    */
    
    double x0_min = env.x_min, x0_max = env.x_max;  // Joint 1 range
    double x1_min = env.y_min, x1_max = env.y_max;  // Joint 2 range
    //double x0_min = -M_PI, x0_max = M_PI;  // Joint 1 range
    //double x1_min = M_PI, x1_max = M_PI;  // Joint 2 range
    // Get bounds and grid size from the cspace object, not from 'this'
    auto x0_bounds = cspace.x0Bounds(); 
    auto x1_bounds = cspace.x1Bounds(); 
    
    double x0_min_grid = x0_bounds.first;
    double x0_max_grid = x0_bounds.second;
    double x1_min_grid = x1_bounds.first;
    double x1_max_grid = x1_bounds.second;
    
    // Get grid size from the cspace object
    auto grid_size = cspace.size(); 
    std::size_t x0_cells = grid_size.first;
    std::size_t x1_cells = grid_size.second;

    collision coll;
    collision::Result result;
    collision::Result result2;
    collision::Result result3;
    bool inCollision = false;
    // number of links:
    int n = manipulator.nLinks();

    // Determine if each cell is in collision or not, and store the values the cspace. This `()` operator comes from DenseArray base class
    for (std::size_t i = 0; i < x0_cells; ++i) {
        for (std::size_t j = 0; j < x1_cells; ++j) {
            //std::cout << "Cell(" << i << "," << j << ")" << std::endl;
            inCollision = false;
            // check cell (i,j)
            
            double theta1 = x0_min + (i + 0.5) * (x0_max - x0_min) / x0_cells; // x0 coord
            double theta2 = x1_min + (j + 0.5) * (x1_max - x1_min) / x1_cells; // x1 coord
            
            //double theta1 = i;
            //double theta2 = j;
            //std::cout << "Theta1: " << theta1 << " Theta2: " << theta2 << std::endl;
            amp::ManipulatorState state;
            state.resize(2);
            state[0] = theta1;
            state[1] = theta2;
            /*
            if(i==0 && j == 0){
                // plot the configuration of the robot
                std::cout << "Theta1: " << theta1 << " Theta2: " << theta2 << std::endl;
                amp::Visualizer::makeFigure(manipulator, state); // plots the robot
            }
            */
            

            /*
            Eigen::Vector2d base = manipulator.getJointLocation(state,0);
            Eigen::Vector2d j1 = manipulator.getJointLocation(state,1);
            Eigen::Vector2d j2 = manipulator.getJointLocation(state,2);

            std::cout << "base location from getJointLocation(): (" 
                    << base.x() << ", " << base.y() << ")" << std::endl;
            std::cout << "j1 location from getJointLocation(): (" 
                << j1.x() << ", " << j1.y() << ")" << std::endl;
            std::cout << "j2 location from getJointLocation(): (" 
                << j2.x() << ", " << j2.y() << ")" << std::endl;
            */

            // NEED TO CHECK EVERY SINGLE POINT ON MANIPULATOR
            // Check for collision
            for(int ii = 0; ii<n; ii++) {
                Eigen::Vector2d point = manipulator.getJointLocation(state, ii);
                Eigen::Vector2d next_point = manipulator.getJointLocation(state,ii+1);

                /*
                std::cout << "point location: (" 
                    << point.x() << ", " << point.y() << ")" << std::endl;
                std::cout << "next_point location: (" 
                    << next_point.x() << ", " << next_point.y() << ")" << std::endl;
                */
               
                result = coll.collisionCheckAllEnv(point, env);
                result2 = coll.collisionCheckAllEnv(next_point, env);
                // if hit add to vector
                if(result.hit) {
                    //std::cout << "Joint 1 Collision at:" << point.transpose() << std::endl;
                    inCollision = true;
                    break;
                } else if(result2.hit) {
                    //std::cout << "Joint 2 Collision at:" << next_point.transpose() << std::endl;
                    inCollision = true;
                    break;
                }
                // if neither point in collision need to check along link
                Eigen::Vector2d dir = (next_point-point).normalized(); // direction from point 1 to point 2
                int steps = x0_cells; // tune this
                double length = (next_point - point).norm();
                double step_size = length / steps;

                for (int s = 1; s < steps; ++s) {  // skip point1, already checked
                    Eigen::Vector2d intermediate = point + dir * (s * step_size);
                    result3 = coll.collisionCheckAllEnv(intermediate, env);
                    if (result3.hit) {
                        //std::cout << "Edge Collision at:" << intermediate.transpose() << " | Between joints: " << ii << "," << ii+1 << std::endl;
                        inCollision = true;
                        break;
                    }
                }
                if(inCollision) {
                    break;
                }
            }
            //std::cout << "cell (" << i << "," << j << ") collision: " << inCollision << std::endl;
            cspace(i,j) = inCollision;
        }
    }

    // Returning the object of type std::unique_ptr<MyGridCSpace2D> can automatically cast it to a polymorphic base-class pointer of type std::unique_ptr<amp::GridCSpace2D>.
    // The reason why this works is not super important for our purposes, but if you are curious, look up polymorphism!
    return cspace_ptr;
}


//////////////////////////////////////////////////////////////

// Override this method for computing all of the boolean collision values for each cell in the cspace
std::unique_ptr<amp::GridCSpace2D> MyPointAgentCSConstructor::construct(const amp::Environment2D& env) {
    // Create an object of my custom cspace type (e.g. MyGridCSpace2D) and store it in a unique pointer. 
    // Pass the constructor parameters to std::make_unique()
    std::unique_ptr<MyGridCSpace2D> cspace_ptr = std::make_unique<MyGridCSpace2D>(m_cells_per_dim, m_cells_per_dim, env.x_min, env.x_max, env.y_min, env.y_max);
    // In order to use the pointer as a regular GridCSpace2D object, we can just create a reference
    MyGridCSpace2D& cspace = *cspace_ptr;
    std::cout << "Constructing C-space for point agent" << std::endl;
    // Get bounds and grid size from the cspace object, not from 'this'
    auto x0_bounds = cspace.x0Bounds(); 
    auto x1_bounds = cspace.x1Bounds(); 
    
    double x0_min_grid = x0_bounds.first;
    double x0_max_grid = x0_bounds.second;
    double x1_min_grid = x1_bounds.first;
    double x1_max_grid = x1_bounds.second;
    
    // Get grid size from the cspace object
    auto grid_size = cspace.size(); 
    std::size_t x0_cells = grid_size.first;
    std::size_t x1_cells = grid_size.second;

    collision coll;
    collision::Result result;



    for (std::size_t i = 0; i < x0_cells; ++i) {
        for (std::size_t j = 0; j < x1_cells; ++j) {
            // check collision values for all obstacles in the space
            bool inCollision = false;
            // assign point in cell to q
            double x = x0_min_grid + (i + 0.5) * (x0_max_grid - x0_min_grid) / x0_cells;
            double y = x1_min_grid + (j + 0.5) * (x1_max_grid - x1_min_grid) / x1_cells;
            Eigen::Vector2d q(x, y);
            // check if point q collides with any obstacles in workspace
            result = coll.collisionCheckAllEnv(q, env);
            // return true if yes, continue if no
            if(result.hit){
                inCollision = true;
            }
            cspace(i,j) = inCollision;
        }
    }

    // Returning the object of type std::unique_ptr<MyGridCSpace2D> can automatically cast it to a polymorphic base-class pointer of type std::unique_ptr<amp::GridCSpace2D>.
    // The reason why this works is not super important for our purposes, but if you are curious, look up polymorphism!
    return cspace_ptr;
}

amp::Path2D MyWaveFrontAlgorithm::planInCSpace(const Eigen::Vector2d& q_init, const Eigen::Vector2d& q_goal, const amp::GridCSpace2D& grid_cspace, bool isManipulator) {
    
    // Implement your WaveFront algorithm here
    amp::Path2D path;
    auto start_cell = grid_cspace.getCellFromPoint(q_init.x(), q_init.y());
    auto end_cell = grid_cspace.getCellFromPoint(q_goal.x(), q_goal.y());

    auto grid_size = grid_cspace.size();
    std::size_t x0_cells = grid_size.first;
    std::size_t x1_cells = grid_size.second;

    std::vector<std::vector<int>> wave(x0_cells, std::vector<int>(x1_cells, -1));

    // build da wavefront grid
    for (std::size_t i = 0; i < x0_cells; ++i) {
        for (std::size_t j = 0; j < x1_cells; ++j) {
            if (grid_cspace(i, j)) { // true = collision
                wave[i][j] = 1; // mark as obstacle
            }
            else {
                wave[i][j] = 0; // free space
            }
        }
    }
    wave[end_cell.first][end_cell.second] = 2; // assign goal cell

    std::queue<std::pair<std::size_t, std::size_t>> q;
    q.push({end_cell.first, end_cell.second});

    std::vector<std::pair<int,int>> neighbor_offsets = {
        {-1, 0}, // left
        { 1, 0}, // right
        { 0,-1}, // down
        { 0, 1}  // up
    };

    while (!q.empty()) {
        auto current = q.front();
        q.pop();
        int cx = current.first; // current cell i
        int cy = current.second; // current cell j

        int current_value = wave[cx][cy]; // current cell value

        // visit all neighbors
        for (auto offset : neighbor_offsets) {
            int nx = cx + offset.first; // next cell i
            int ny = cy + offset.second; // next cell j

            // check bounds
            if (nx >= 0 && nx < x0_cells && ny >= 0 && ny < x1_cells) {
                // only propagate to free cells (0)
                if (wave[nx][ny] == 0) {
                    wave[nx][ny] = current_value + 1; // increment distance
                    q.push({nx, ny});
                }
            }
        }
    }

    // now to do the planning
    

    
    std::pair<int,int> current = {static_cast<int>(start_cell.first), static_cast<int>(start_cell.second)};
    path.waypoints.push_back(q_init); // Start configuration in continuous space

    while (wave[current.first][current.second] != 2) { // Until we reach the goal
        int cx = current.first;
        int cy = current.second;

        std::pair<int,int> next = current;
        int min_val = wave[cx][cy];

        // Check all 8 neighbors
        for (auto offset : neighbor_offsets) {
            int nx = cx + offset.first;
            int ny = cy + offset.second;

            if (nx >= 0 && nx < x0_cells && ny >= 0 && ny < x1_cells) {
                int val = wave[nx][ny]; // value of test cell
                if (val > 1 && val < min_val) { // Must be part of valid wavefront
                    min_val = val;
                    next = {nx, ny}; 
                }
            }
        }

        // If stuck, break
        if (next == current) {
            std::cerr << "Wavefront path extraction stuck!" << std::endl;
            break;
        }

        // Convert grid cell to continuous space coordinates
        double x = grid_cspace.x0Bounds().first + (next.first + 0.5) * (grid_cspace.x0Bounds().second - grid_cspace.x0Bounds().first) / x0_cells;
        double y = grid_cspace.x1Bounds().first + (next.second + 0.5) * (grid_cspace.x1Bounds().second - grid_cspace.x1Bounds().first) / x1_cells;
        path.waypoints.push_back(Eigen::Vector2d(x, y));

        current = next;
    }
    path.waypoints.push_back(q_goal);

    if (isManipulator) {
        Eigen::Vector2d bounds0 = Eigen::Vector2d(0.0, 0.0);
        Eigen::Vector2d bounds1 = Eigen::Vector2d(2*M_PI, 2*M_PI);
        amp::unwrapWaypoints(path.waypoints, bounds0, bounds1);
    }
    return path;
}
