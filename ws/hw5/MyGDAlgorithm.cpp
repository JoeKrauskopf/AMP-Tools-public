#include "MyGDAlgorithm.h"

// Implement your plan method here, similar to HW2:
amp::Path2D MyGDAlgorithm::plan(const amp::Problem2D& problem) {
    amp::Path2D path;
    // q(0) = q_star
    q = problem.q_init;
    path.waypoints.push_back(q);
    int i=1;
    int max_iterations = 10000;
    double epsilon = 1e-4; // tolerance for reaching local minimum


    MyPotentialFunction potential(problem.q_goal, problem, d_star, zetta);
    Eigen::Vector2d grad = potential.getGradient(q);

    while(grad.norm() >= epsilon && i < max_iterations) {
        grad = potential.getGradient(q);
        Eigen::Vector2d q_next = q - eta*grad;
        
        // check if reached goal
        if((problem.q_goal - q_next).norm() <= 1e-4){
            // reached goal
            path.waypoints.push_back(q_next);
            path.waypoints.push_back(problem.q_goal);
            break;
        }
        q = q_next;
        path.waypoints.push_back(q);
        i++;
    }
    /*
    while(norm(deltaU(q_i)) <= epsilon)  {
        q(i+1) = q(i) - alpha(i)*deltaU(q_i)
        i++
    }
    */
    return path;
}
