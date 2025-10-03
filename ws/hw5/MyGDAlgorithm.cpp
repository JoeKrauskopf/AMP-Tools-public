#include "MyGDAlgorithm.h"
#include <iostream>
#include "collisionCheck.h"

// maybe add pertubation if getting stuck

// Implement your plan method here, similar to HW2:
amp::Path2D MyGDAlgorithm::plan(const amp::Problem2D& problem) {
    amp::Path2D path;
    // q(0) = q_star
    q = problem.q_init;
    path.waypoints.push_back(q);
    //double alpha = 0.1;
    int i=1;
    int max_iterations = 1000;
    double epsilon = 1e-3; // tolerance for reaching local minimum

    collision coll;
    collision::Result result;
    bool inCollision = false;

    char escapeOpt = 'A';


    MyPotentialFunction potential(problem.q_goal, problem, d_star, zetta, Q_star, eta);
    Eigen::Vector2d grad = potential.getGradient(q);
    std::cout << "Starting gradient descent..." << std::endl;
    std::cout << "Initial position: " << q.transpose() << std::endl;


    while(grad.norm() >= epsilon && i < max_iterations) {
        grad = potential.getGradient(q);
        double alpha = 0.1 / (1 + grad.norm());
        Eigen::Vector2d q_next = q - alpha*grad;
        Eigen::Vector2d q_og = q;

        if (grad.norm() < epsilon && (problem.q_goal - q).norm() > 0.25) {
            std::cout << "Detected local minimum at iteration " << i << std::endl;
            // print local min grad
            std::cout << "Local Min Grad: " << grad.transpose() << std::endl;
            switch (escapeOpt)
            {
                case 'A': {
                    // option a: random pertubation:
                    Eigen::Vector2d perturbation = Eigen::Vector2d::Random().normalized() * 0.5; 
                    q = q_og + perturbation;
                    // check if this new point collides
                    result = coll.collisionCheckAll(q, problem);
                    while(result.hit) {
                        Eigen::Vector2d perturbation = Eigen::Vector2d::Random().normalized() * 0.5; 
                        q = q_og + perturbation;
                        // check if this new point collides
                        result = coll.collisionCheckAll(q, problem);
                    }

                    
                    path.waypoints.push_back(q);
                    grad = potential.getGradient(q);

                    std::cout << "Applied random perturbation: " << perturbation.transpose() << std::endl;
                    continue;
                    // if continues to fall into local minima try larger random pertubation but check along edge for collisions

                }
                
                case 'B': {
                    // option b: trangential pertubation
                    Eigen::Vector2d orth(-grad.y(), grad.x()); // 90-degree rotated vector
                    
                    orth.normalize();
                    std::cout << "Rotated Grad: " << orth.transpose() << std::endl;
                    q = q_og + 0.5 * orth; // step sideways
                    path.waypoints.push_back(q);
                    grad = potential.getGradient(q);

                    std::cout << "Applied tangential perturbation: " << orth.transpose() << std::endl;
                    continue;
                }

                case 'C': {
                    // bug 2 code
                    std::cout << "Detected local minimum, applying simple wall-following..." << std::endl;

                    // Local Bug2 variables
                    Eigen::Vector2d q_bug = q;           // start at local minimum
                    Eigen::Vector2d q_prev = q - grad.normalized() * 0.01; // small previous step
                    Eigen::Vector2d q_goal = problem.q_goal;
                    bool boundaryFollowing = true;
                    int boundarySteps = 0;
                    int maxBoundarySteps = 1000;
                    double stepSize = 0.05;
                    char direction = 'R'; // try right-hand wall following

                    collision coll;

                }
            
                default: {
                    break;
                }
            }
           
        }


        double U = potential(q); // potential at current position

        // Debug output
        /*
        std::cout << "Step: " << i << std::endl;
        std::cout << "  Current q: " << q.transpose() << std::endl;
        std::cout << "  alpha: " << alpha << std::endl;
        std::cout << "  Gradient: " << grad.transpose() << std::endl;
        std::cout << "  Gradient norm: " << grad.norm() << std::endl;
        std::cout << "  Potential U(q): " << U << std::endl;
        std::cout << "  Next q: " << q_next.transpose() << std::endl;
        std::cout << "  Distance to goal: " << (problem.q_goal - q_next).norm() << std::endl;
        */
        
        // check if reached goal
        if((problem.q_goal - q_next).norm() <= 0.25){
            // reached goal
            path.waypoints.push_back(q_next);
            path.waypoints.push_back(problem.q_goal);
            break;
        }
        q = q_next;
        //grad = potential.getGradient(q);
        path.waypoints.push_back(q);
        i++;

    }
    if (i >= max_iterations) {
        std::cout << "Reached maximum iterations without converging." << std::endl;
    } else if (grad.norm() < epsilon) {
        std::cout << "Gradient norm below threshold. Stopping." << std::endl;
    }

    return path;
}
