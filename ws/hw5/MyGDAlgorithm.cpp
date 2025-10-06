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
    std::vector<Eigen::Vector2d> localMinima; 
    double minPert = 0.1;
    double lastPert = minPert; // initialize last pert

    char escapeOpt = 'B';


    MyPotentialFunction potential(problem.q_goal, problem, d_star, zetta, Q_star, eta);
    Eigen::Vector2d grad = potential.getGradient(q);
    std::cout << "Starting gradient descent..." << std::endl;
    std::cout << "Initial position: " << q.transpose() << std::endl;


    while(grad.norm() >= epsilon && i < max_iterations) {
        grad = potential.getGradient(q);
        double alpha = 0.1 / (1 + grad.norm());
        Eigen::Vector2d q_og = q;

        Eigen::Vector2d q_next = q - alpha*grad;

        // try adding random pert to q_next
        //Eigen::Vector2d perturbation_q_next = Eigen::Vector2d::Random().normalized() * 0.001;
        //q_next += perturbation_q_next;

        

        if (grad.norm() < epsilon && (problem.q_goal - q).norm() > 0.25) {
            Eigen::Vector2d localMinLoc = q;
            std::cout << "Detected local minimum at iteration " << i << std::endl;
            // add vector of local minima points
            // Check if this local minimum has been visited before
            bool revisited = false;
            for (const auto& prevMin : localMinima) {
                if ((localMinLoc - prevMin).norm() < 1e-2) {  // threshold for "same" minimum
                    revisited = true;
                    break;
                }
            }

            // Store the current local minimum
            localMinima.push_back(localMinLoc);

            if (revisited) {
                std::cout << "This local minimum has been visited before. Consider increasing perturbation magnitude." << std::endl;
                // you can increase perturbation magnitude here
            }

            // print local min grad
            //std::cout << "Local Min Grad: " << grad.transpose() << std::endl;
            switch (escapeOpt)
            {
                case 'A': {
                    // Random perturbation with edge collision checking and adaptive magnitude
                    int numPerts = 100;
                    for(int j =0; j<numPerts; j++){
                        double localPert;
                        // check if we've been in this local minima before
                        if(revisited) {
                            localPert = lastPert;
                        }else {
                            localPert = minPert;
                        }
                        Eigen::Vector2d perturbation = Eigen::Vector2d::Random().normalized() * localPert;
                        Eigen::Vector2d candidate = q + perturbation;

                        // ---- Collision check along the edge ----
                        int numSamples = 10;
                        bool validPerturb = true;
                        for (int s = 1; s <= numSamples; ++s) {
                            Eigen::Vector2d samplePoint = q + (perturbation * s / numSamples);
                            result = coll.collisionCheckAll(samplePoint, problem);
                            if (result.hit) {
                                validPerturb = false;
                                break;
                            }
                        }
                        if (!validPerturb) {
                        // Gradually increase magnitude if stuck
                        localPert = std::min(localPert * 1.5, 10.0);
                        } else {
                            // Success — apply perturbation
                            q = candidate;
                            path.waypoints.push_back(q);
                            grad = potential.getGradient(q);
                            std::cout << "Applied random perturbation (edge-checked): "
                                    << perturbation.transpose()
                                    << " | magnitude: " << localPert
                                    << " | new position: " << q.transpose() << std::endl;
                        }
                    }
                    continue; // resume gradient descent
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
                    // try to wall follow left then right to escape
                    bool escaped = false;
                    for(int dir=1; dir<3; dir++) {
                        // dir = 1 left, 2 = right
                        
                    }


                    continue; // continue main gradient descent loop
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
