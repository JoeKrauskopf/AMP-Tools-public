#include "CSpaceSkeleton.h"
#include "collisionCheck.h"
#include "AMPCore.h"
#include "hw/HW4.h"
#include "tools/Visualizer.h"

// Override this method for returning whether or not a point is in collision

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

// Override this method for computing all of the boolean collision values for each cell in the cspace
std::unique_ptr<amp::GridCSpace2D> MyManipulatorCSConstructor::construct(const amp::LinkManipulator2D& manipulator, const amp::Environment2D& env) {
    // Create an object of my custom cspace type (e.g. MyGridCSpace2D) and store it in a unique pointer. 
    // Pass the constructor parameters to std::make_unique()
    std::unique_ptr<MyGridCSpace2D> cspace_ptr = std::make_unique<MyGridCSpace2D>(m_cells_per_dim, m_cells_per_dim, env.x_min, env.x_max, env.y_min, env.y_max);
    // In order to use the pointer as a regular GridCSpace2D object, we can just create a reference
    MyGridCSpace2D& cspace = *cspace_ptr;

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
    
    double x0_min = 0, x0_max = 2*M_PI;  // Joint 1 range
    double x1_min = 0, x1_max = 2*M_PI;  // Joint 2 range
    
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
            if(i==1 && j == 9){
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
                int steps = 200; // tune this
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
