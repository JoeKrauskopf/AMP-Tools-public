#include "MyBugAlgorithm.h"
#include "collisionCheck.h"
#include <iostream>
#include <limits>
#include <algorithm>

amp::Path2D MyBugAlgorithm::plan(const amp::Problem2D& problem) {
    amp::Path2D path;

    Eigen::Vector2d q = problem.q_init;
    Eigen::Vector2d q_start = problem.q_init;
    Eigen::Vector2d q_previous = problem.q_init;
    Eigen::Vector2d q_goal = problem.q_goal;
    const double tol = 1e-4;
    const double stepSize = 0.04;  // Distance to move toward goal
    const double safeDist = 0.04;  // Distance for boundary following
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
    Q.resize(1, 2);
    
    path.waypoints.push_back(q);
    
    collision coll;
    int steps = 0;
    int i = 1;
    int boundarySteps = 0;  // Count steps while following boundary
    
    while (!goalReached && steps < maxSteps) {
        steps++;
        
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
            Eigen::Vector2d q_next = q + stepSize * dir;

            
            
            // Check for collision
            collision::Result result = coll.collisionCheckAll(q_next, problem);
            
            if (!result.hit) {
                // No collision, move toward goal

                // Store current point for boundary stuff
                q_previous = q;

                path.waypoints.push_back(q_next);
                q = q_next;
                
            } else {
                // Hit obstacle, start boundary following
                std::cout << "Hit obstacle: " << i << " at q_H: " << q.transpose() << std::endl;
                boundaryFollowing = true;
                collide = true;
                loopCompleted = false;
                boundarySteps = 0;
                i++;
                
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
                loopCompleted = true;
            }
            
            // If loop is completed and we're now at the best leave point, exit boundary following
            if (loopCompleted) {
                Eigen::Vector2d bestLeavePoint = q_L.row(i-1).transpose();
                if ((q - bestLeavePoint).norm() <= tol) {
                    std::cout << "Reached best q_L: " << q.transpose() << ", exiting boundary following" << std::endl;
                    boundaryFollowing = false;
                    collide = false;
                    
                    continue;
                }
            }
            
            // Move along boundary using wall-following (CCW or CW based on direction)
            Eigen::Vector2d nextBoundaryPoint = getNextBoundaryPoint(q, q_previous, problem, coll, 
                                                                    safeDist, checkDist);
            
            if ((nextBoundaryPoint - q).norm() > tol) {  // Valid move found
                // Store as previous point
                q_previous = q;
                path.waypoints.push_back(nextBoundaryPoint);
                q = nextBoundaryPoint;
            } else {
                std::cout << "Stuck in boundary following!" << std::endl;
                break;
            }
        }
    }
    
    if (steps >= maxSteps) {
        std::cout << "Maximum steps reached without finding goal" << std::endl;
    }
    
    return path;
}



Eigen::Vector2d MyBugAlgorithm::getNextBoundaryPoint(const Eigen::Vector2d& current,
                                                    const Eigen::Vector2d& previous,
                                                    const amp::Problem2D& problem,
                                                    collision& coll,
                                                    double safeDist,
                                                    double checkDist) {
    
    // 8 neighbors around q 
    std::vector<Eigen::Vector2d> neighbors = { Eigen::Vector2d(0, 1), // 0: North 
    Eigen::Vector2d(1, 1).normalized(), // 1: NorthEast 
    Eigen::Vector2d(1, 0), // 2: East 
    Eigen::Vector2d(1, -1).normalized(), // 3: SouthEast 
    Eigen::Vector2d(0, -1), // 4: South 
    Eigen::Vector2d(-1, -1).normalized(), // 5: SouthWest 
    Eigen::Vector2d(-1, 0), // 6: West 
    Eigen::Vector2d(-1, 1).normalized() // 7: NorthWest 
    };
    // map indices to direction names 
    std::vector<std::string> dirNames = { "North", "NorthEast", "East", "SouthEast", "South", "SouthWest", "West", "NorthWest" };

    // find closest matching previous direction
    Eigen::Vector2d prevDirVec = (current - previous).normalized();
    int currentDir = 0;
    double bestMatch = -2.0; // worst possible dot product
    for (int i = 0; i < 8; i++) {
        double dotProduct = prevDirVec.dot(neighbors[i]);
        if (dotProduct > bestMatch) {
            bestMatch = dotProduct;
            currentDir = i;
        }
    }

    // check all 8 directions for collisions
    std::vector<bool> collisions(8, false);
    for (int i = 0; i < 8; i++) {
        Eigen::Vector2d testPoint = current + checkDist * neighbors[i];
        collision::Result checkDir = coll.collisionCheckAll(testPoint, problem);
        collisions[i] = checkDir.hit;
    }

    std::vector<int> validDirs;
    // store collisions to find valid directions to move
    for (int i = 0; i < 8; i++) {
        Eigen::Vector2d movePoint = current + safeDist * neighbors[i];
        collision::Result moveDir = coll.collisionCheckAll(movePoint, problem);
        if (!moveDir.hit) {
            validDirs.push_back(i);
        }
    }
    //std::cout << "Current Direction: " << currentDir << std::endl;

    // do a wallhugging check
    int leftDir = (currentDir + 7) % 8;   // CCW from current (left side)
    int rightDir = (currentDir + 1) % 8;  // CW from current (right side)
    bool leftCollides = collisions[leftDir];
    bool currentCollides = collisions[currentDir];
    bool rightCollides = collisions[rightDir];
    bool atCorner = false;

    // do a vertex check
    if (direction == 'L') {
        atCorner = (!leftCollides && !currentCollides);
        //std::cout << "Hit corner: " << atCorner << std::endl;
    } else {
        atCorner = (!rightCollides && !currentCollides);
    }
    
    //std::cout << "Left Collide: " << leftCollides << " Right Collide: " << rightCollides << " Current Collide: " << currentCollides << " Corner: " << atCorner << std::endl;
    // calculate the next valid point to move
    int nextDir = currentDir;

    
    if(atCorner) {
        //std::cout << "Reached corner at " << current. transpose() << std::endl;
        // At vertex - turn left (CW) since traveling CCW around obstacle
       for (int offset = -1; offset >= -3; offset--) {
           int testDir = (currentDir + offset + 8) % 8;
           if (std::find(validDirs.begin(), validDirs.end(), testDir) != validDirs.end()) {
               nextDir = testDir;
               break;
           }
       }
       /*
       std::cout << "Next chosen direction: " 
          << dirNames[nextDir] 
          << " (index " << nextDir << ")" << std::endl;
          */
    } else if (leftCollides || currentCollides) {
        
        // Wall hugging - try to continue straight, then turn left if blocked
       if (std::find(validDirs.begin(), validDirs.end(), currentDir) != validDirs.end()) {
           nextDir = currentDir;
       } else {
           // Turn right (CCW) to avoid obstacle
           for (int offset = 1; offset <= 3; offset++) {
               int testDir = (currentDir + offset) % 8;
               if (std::find(validDirs.begin(), validDirs.end(), testDir) != validDirs.end()) {
                   nextDir = testDir;
                   break;
               }
           }
       }
    }
    // return
    return current + safeDist * neighbors[nextDir];

    
}
