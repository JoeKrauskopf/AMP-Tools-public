#include "MyBugAlgorithm.h"
#include "collisionCheck.h"
#include <iostream>
#include <limits>
#include <algorithm>

amp::Path2D MyBugAlgorithm::plan(const amp::Problem2D& problem) {
    amp::Path2D path;

    q = problem.q_init;
    Eigen::Vector2d q_start = problem.q_init;
    q_previous = problem.q_init;
    Eigen::Vector2d q_goal = problem.q_goal;
    
    const double tol = 1e-2;
    const double minStepSize = 0.05;  // Distance to move toward goal
    const double maxStepSize = 0.2; // maximum distance to travel toward goal
    double currStepSize = maxStepSize;
    
    const double safeDist = 0.05;  // Distance for boundary following
    const double checkDist = 0.05; // Distance for collision checking
    const int maxSteps = 10000;
    
    // Initialize member variables
    goalReached = false;
    collide = false;
    boundaryFollowing = false;
    loopCompleted = false;
    
    // Initialize matrices
    q_L.resize(0, 2);  // Will store leave points
    q_H.resize(0, 2);  // Will store hit points
    
    path.waypoints.push_back(q);
    
    collision coll;
    collision::Result result;
    int steps = 0;
    int i = 1;
    int boundarySteps = 0;  // Count steps while following boundary
    
    while (!goalReached && steps < maxSteps) {
        steps++;
        //std::cout << "step: " << steps << std::endl;
        
        // Check if goal is reached
        if ((q_goal - q).norm() <= 1e-1) {
            std::cout << "Goal Reached!" << std::endl;
            goalReached = true;
            path.waypoints.push_back(q_goal);
            break;
        }
        
        if (!boundaryFollowing) {
            // PHASE 1: Move toward goal
            Eigen::Vector2d dir = (q_goal - q).normalized();
            q_next = q + currStepSize * dir;

            
            
            // Check for collision
            result = coll.collisionCheckAll(q_next, problem);
            
            if (!result.hit) {
                // No collision, move toward goal

                // Store current point for boundary stuff
                q_previous = q;

                path.waypoints.push_back(q_next);
                q = q_next;
                
            } else {
                currStepSize -= 0.01;
                if((q_next-q).norm() <= safeDist) {
                    currStepSize = maxStepSize;
                    // Hit obstacle, start boundary following
                    //std::cout << "Hit obstacle: " << i << " at q_H: " << q.transpose() << std::endl;
                    boundaryFollowing = true;
                    collide = true;
                    loopCompleted = false;
                    i++;
                    boundarySteps = 0;

                    // Add hit point to q_H
                    q_H.conservativeResize(i + 1, 2);
                    q_H.row(i-1) = q.transpose();
                    
                    // Initialize leave point as current hit point
                    q_L.conservativeResize(i + 1, 2);
                    q_L.row(i-1) = q.transpose();
                    
                    
                    
                    std::cout << "Starting boundary following (" << direction << " direction)" << std::endl;
                    std::cout << "Initial q_L: " << q_L.row(i-1) << " (dist to goal: " 
                            << (q_goal - q).norm() << ")" << std::endl;
                }
            }
                
        } else {
            // PHASE 2: Boundary following
            boundarySteps++;
            //std::cout << "Current boundaryStep: " <<boundarySteps << " Current pos: " << q.transpose() << std::endl;
            double currentDistToGoal = (q_goal - q).norm();
            double bestLeaveDistToGoal = (q_goal - q_L.row(i-1).transpose()).norm();
            
            // Always update leave point if current position is closer to goal
            if (currentDistToGoal < bestLeaveDistToGoal) {
                q_L.row(i-1) = q.transpose();
                //std::cout << "Updated q_L to: " << q.transpose() 
                        //<< " (dist to goal: " << currentDistToGoal << ")" << std::endl;
            }
            
            // Check if we've completed a full loop (returned to q_H)
            if (!loopCompleted && (q - q_H.row(i-1).transpose()).norm() <= 1e-1 && boundarySteps > 10) { //HAD TO MASSIVELY REDUCE TOLERANCE
                std::cout << "Full loop completed! Returned to q_H: " << q.transpose() << std::endl;
                std::cout << "Moving to best q_L: " << q_L.row(i-1) << std::endl;
                loopCompleted = true;
            }
            
            // If loop is completed and we're now at the best leave point, exit boundary following
            if (loopCompleted) {
                Eigen::Vector2d bestLeavePoint = q_L.row(i-1).transpose();
                if ((q - bestLeavePoint).norm() <= 1e-1) {
                    std::cout << "Reached best q_L: " << q.transpose() << ", exiting boundary following" << std::endl;
                    boundaryFollowing = false;
                    collide = false;
                    
                    continue;
                }
            }
            
            // Move along boundary using wall-following (CCW or CW based on direction)
            Eigen::Vector2d q_next_boundary;
            
            // Follow boundary using wall-following
            q_next_boundary = followBoundary(q, q_previous, problem, coll, safeDist, checkDist, direction);
            
            
            
            if ((q_next_boundary - q).norm() > tol) {
                //std::cout <<"Valid q_next_boundary, pushing to waypoint" << std::endl;
                q_previous = q;
                path.waypoints.push_back(q_next_boundary);
                q = q_next_boundary;
            } else {
                std::cout << "Stuck in boundary following!" << std::endl;
                break;
            }
        }
    }
    
    if (steps >= maxSteps) {
        std::cout << "Maximum steps reached without finding goal" << std::endl;
        std::cout << "Last recorded pos: " << q.transpose() << std::endl;
    }
    
    return path;
}

// THIS NEEDS MORE WORK BUT CLOSE
amp::Path2D MyBugAlgorithm::planBUG2(const amp::Problem2D& problem) {
    amp::Path2D path;

    q = problem.q_init;
    Eigen::Vector2d q_start = problem.q_init;
    q_previous = problem.q_init;
    Eigen::Vector2d q_goal = problem.q_goal;
    const Eigen::Vector2d mLine = (q_goal - q_start);
    

    
    const double tol = 1e-1;
    const double minStepSize = 0.05;  // Distance to move toward goal
    const double maxStepSize = 0.2; // maximum distance to travel toward goal
    double currStepSize = maxStepSize;
    
    const double safeDist = 0.05;  // Distance for boundary following
    const double checkDist = 0.05; // Distance for collision checking
    const int maxSteps = 10000;
    
    // Initialize member variables
    bool onMline = false;
    goalReached = false;
    collide = false;
    boundaryFollowing = false;
    loopCompleted = false;
    
    // Initialize matrices
    q_L.resize(0, 2);  // Will store leave points
    q_H.resize(0, 2);  // Will store hit points
    
    path.waypoints.push_back(q);
    
    collision coll;
    collision::Result result;
    int steps = 0;
    int i = 1;
    int boundarySteps = 0;  // Count steps while following boundary
    
    while (!goalReached && steps < maxSteps) {
        steps++;
        //std::cout << "step: " << steps << std::endl;
        
        // Check if goal is reached
        if ((q_goal - q).norm() <= tol) {
            std::cout << "Goal Reached!" << std::endl;
            goalReached = true;
            path.waypoints.push_back(q_goal);
            break;
        }
        
        if (!boundaryFollowing) {
            // PHASE 1: Move toward goal
            Eigen::Vector2d dir = (q_goal - q).normalized();
            q_next = q + currStepSize * dir;

            
            
            // Check for collision
            result = coll.collisionCheckAll(q_next, problem);
            
            if (!result.hit) {
                // No collision, move toward goal

                // Store current point for boundary stuff
                onMline = true;
                q_previous = q;

                path.waypoints.push_back(q_next);
                q = q_next;
                
            } else {
                currStepSize -= 0.01;
                if((q_next-q).norm() <= safeDist) {
                    currStepSize = maxStepSize;
                    // Hit obstacle, start boundary following
                    //std::cout << "Hit obstacle: " << i << " at q_H: " << q.transpose() << std::endl;
                    onMline = false;
                    boundaryFollowing = true;
                    collide = true;
                    loopCompleted = false;
                    i++;
                    boundarySteps = 0;

                    // Add hit point to q_H
                    q_H.conservativeResize(i + 1, 2);
                    q_H.row(i-1) = q.transpose();
                    
                    // Initialize leave point as current hit point
                    q_L.conservativeResize(i + 1, 2);
                    q_L.row(i-1) = q.transpose();
                    
                    
                    
                    std::cout << "Starting boundary following (" << direction << " direction)" << std::endl;
                    std::cout << "Initial q_L: " << q_L.row(i-1) << " (dist to goal: " 
                            << (q_goal - q).norm() << ")" << std::endl;
                }
            }
                
        } else {
            // PHASE 2: Boundary following
            boundarySteps++;
            //std::cout << "Current boundaryStep: " <<boundarySteps << " Current pos: " << q.transpose() << std::endl;
            double currentDistToGoal = (q_goal - q).norm();
            double bestLeaveDistToGoal = (q_goal - q_L.row(i-1).transpose()).norm();
            
            // Always update leave point if current position is closer to goal
            if (currentDistToGoal < bestLeaveDistToGoal) {
                q_L.row(i-1) = q.transpose();
                //std::cout << "Updated q_L to: " << q.transpose() 
                        //<< " (dist to goal: " << currentDistToGoal << ")" << std::endl;
            }
            bool onMline = isOnMline(q, q_start, q_goal, 3e-1);
            

            if (onMline && (q_goal - q).norm() < (q_goal - q_H.row(i-1).transpose()).norm() && boundarySteps > 10) {
                // reencountered m-line
                std::cout << "re-encountered m-line " << q.transpose() << std::endl;
                //std::cout << "Moving to best q_L: " << q_L.row(i-1) << std::endl;
                //loopCompleted = true;
                boundaryFollowing = false;
                collide = false;
                continue;
            }

            /*
            // Check if we've completed a full loop (returned to q_H)
            if (!loopCompleted && (q - q_H.row(i-1).transpose()).norm() <= tol && boundarySteps > 10) { //HAD TO MASSIVELY REDUCE TOLERANCE
                std::cout << "Full loop completed! Returned to q_H: " << q.transpose() << std::endl;
                std::cout << "Moving to best q_L: " << q_L.row(i-1) << std::endl;
                loopCompleted = true;
            }
            */
            
            
            // If loop is completed and we're now at the best leave point, exit boundary following
            if (loopCompleted) {
                
                std::cout << "Reached best q_L: " << q.transpose() << ", exiting boundary following" << std::endl;
                boundaryFollowing = false;
                collide = false;
                    
                continue;
                /*
                Eigen::Vector2d bestLeavePoint = q_L.row(i-1).transpose();
                if ((q - bestLeavePoint).norm() <= 1e-1) {
                    std::cout << "Reached best q_L: " << q.transpose() << ", exiting boundary following" << std::endl;
                    boundaryFollowing = false;
                    collide = false;
                    
                    continue;
                }
                */
            }
            
            // Move along boundary using wall-following (CCW or CW based on direction)
            Eigen::Vector2d q_next_boundary;
            
            // Follow boundary using wall-following
            q_next_boundary = followBoundary(q, q_previous, problem, coll, safeDist, checkDist, direction);
            
            
            
            if ((q_next_boundary - q).norm() > 1e-3) {
                //std::cout <<"Valid q_next_boundary, pushing to waypoint" << std::endl;
                q_previous = q;
                path.waypoints.push_back(q_next_boundary);
                q = q_next_boundary;
            } else {
                std::cout << "Stuck in boundary following!" << std::endl;
                break;
            }
        }
    }
    
    if (steps >= maxSteps) {
        std::cout << "Maximum steps reached without finding goal" << std::endl;
        std::cout << "Last recorded pos: " << q.transpose() << std::endl;
    }
    
    return path;
}




// Simple wall following - keeps obstacle on the right side (for CCW traversal)
Eigen::Vector2d MyBugAlgorithm::followBoundary(const Eigen::Vector2d& current,
                                            const Eigen::Vector2d& previous,
                                            const amp::Problem2D& problem,
                                            collision& coll,
                                            double stepSize,
                                            double radius,
                                            char direction) {
    Eigen::Vector2d dir = (current - previous).normalized();
    //std::cout <<"Dir: " << dir.transpose() << std::endl;
    int numVec = 16;
    // SWAP TO CIRCLE BASED ON ON RADIUS EPISLON WITH RAYS INSIDE
    std::vector<Eigen::Vector2d> directions;
    std::vector<bool> collision_free;
    // define a circle of CW normalized unit vectors with vec0 = dir and sweeping 360 deg
    for (int i = 0; i < numVec; i++) {
        double angle = 2.0 * M_PI * i / numVec;
        
        // Rotate the forward direction by the angle
        double cos_a = cos(angle);
        double sin_a = sin(angle);
        
        Eigen::Vector2d rotated_dir;
        rotated_dir(0) = dir(0) * cos_a - dir(1) * sin_a;
        rotated_dir(1) = dir(0) * sin_a + dir(1) * cos_a;
        
        directions.push_back(rotated_dir.normalized());
        //std::cout << "Test Dir " << i << ": " << directions[i].transpose() << std::endl;
    }
    // CHECK FOR COLLISIONS CCW VS CW FOR RIGHT VS LEFT TURNING AMONG ALL VECTORS
    collision_free.resize(numVec);
    for (int i = 0; i < numVec; i++) {
        
        Eigen::Vector2d test_point = current + stepSize * directions[i];
        if(i==0) {
            //std::cout <<"test_point[0]:" << test_point.transpose() << std::endl;
        }
        collision::Result result = coll.collisionCheckAll(test_point, problem);
        collision_free[i] = !result.hit;
    }
    //std::cout << "First collision vector: " <<directions[0].transpose() <<std::endl;
    /*
    std::cout << "collision_free: ";
    for (size_t i = 0; i < collision_free.size(); i++) {
        std::cout << collision_free[i] << " ";
    }
    std::cout << std::endl;
    */
    bool atEdge = false;
    int best_index;
     std::vector<int> priority_order;
    // need to check if were at a corner
    if(!collision_free[0]) {
        atEdge = true;
    }

    // corner logic
    if(atEdge) {
        if (direction == 'R') {
            // Right-hand corner following (CCW search)
            // Start from forward direction and search clockwise for first collision-free direction
            best_index = numVec+1;
            for(int i = 0; i<numVec; i++) {
                int index = (numVec-i) % numVec; // index from numVec-i
                if(collision_free[index] && index < best_index) {
                    best_index = index;
                }
            }
            
        } else {
            // Left-hand corner following (CW search)
            // Start from forward direction and search counter-clockwise for first collision-free direction
            best_index = -1;
            for(int i = 0; i<numVec; i++) {
                if(collision_free[i] && i>best_index) {
                    best_index = i;
                }
            }
            
        }
    } else {
        if (direction == 'L') {
            // Left-hand wall following (CCW search)
            // Start from forward direction and search clockwise for first collision-free direction
            best_index = numVec+1;
            for(int i = 1; i<(numVec/2); i++) {
                priority_order.push_back(i);
            }
            priority_order.push_back(0);
            priority_order.push_back(numVec-1);
            for(int j : priority_order) {
                if (collision_free[j]) {
                    best_index = j;
                    //std::cout << "Right-hand following: selected index " << idx << std::endl;
                    break;
                }
            }
            
        } else {
            // Right-hand wall following (CW search)
            // Start from forward direction and search counter-clockwise for first collision-free direction
            //std::cout <<"Starting Right-hand wall hugging" << std::endl;
            best_index = numVec+1;
            for(int i = numVec-1; i>((numVec/2)); i--) {
                priority_order.push_back(i);
            }
            priority_order.push_back(0);
            priority_order.push_back(1);

            for(int j : priority_order) {
                if (collision_free[j]) {
                    best_index = j;
                    //std::cout << "Right-hand following: selected index " << idx << std::endl;
                    break;
                }
            }
            
            //std::cout <<"Best dir idx: " << best_index <<std::endl;

            
        }

    }
    
    // If no collision-free direction found, try to back away slightly IGNORE FOR NOW
    /*
    if (best_index == -1) {
        // Emergency: try to move away from obstacle
        Eigen::Vector2d away_dir = -dir;
        return current + 0.01 * away_dir;
    }
    */
    
    // Move in the selected direction
    
    Eigen::Vector2d nextDir = directions[best_index];
    //std::cout << "Best direction is " << nextDir.transpose() << std::endl;
    return current + stepSize * nextDir;
}


// Move toward the leave point while avoiding obstacles
Eigen::Vector2d MyBugAlgorithm::moveTowardLeavePoint(const Eigen::Vector2d& current,
                                                     const Eigen::Vector2d& previous,
                                                     const Eigen::Vector2d& leavePoint,
                                                     const amp::Problem2D& problem,
                                                     collision& coll,
                                                     double stepSize,
                                                     double radius,
                                                     char direction) {
    
    // Try to move directly toward leave point
    Eigen::Vector2d dir = (leavePoint - current).normalized();
    Eigen::Vector2d directPoint = current + stepSize * dir;
    
    collision::Result result = coll.collisionCheckAll(directPoint, problem);
    if (!result.hit) {
        return directPoint;
    }
    
    // If direct path blocked, continue boundary following
    return followBoundary(current, previous, problem, coll, stepSize, radius, direction);
}


bool MyBugAlgorithm::isOnMline(const Eigen::Vector2d& current,
                    const Eigen::Vector2d& start,
                    const Eigen::Vector2d& end,
                    double tol) {
    Eigen::Vector2d qVec = current-start;
    Eigen::Vector2d mLine = end-start;

    double cross = std::abs(mLine.x() * qVec.y() - mLine.y() * qVec.x());
    double dot = qVec.dot(mLine);
    if (cross > tol) {
        return false; // colinearity
    }
    //check if ahead or behind mline
    if (dot < -tol) {
        return false; 
    }
    if (dot > mLine.squaredNorm() + tol) {
        return false;
    }
    return true;
}

