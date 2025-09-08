#include "MyBugAlgorithm.h"
#include "collisionCheck.h"
#include <iostream>
#include <limits>

amp::Path2D MyBugAlgorithm::plan(const amp::Problem2D& problem) {
    amp::Path2D path;

    Eigen::Vector2d q = problem.q_init;
    Eigen::Vector2d q_next;
    Eigen::Vector2d q_goal = problem.q_goal;
    const double tol = 1e-4;
    int i = 1;
    int count = 0;
    int maxCount = 1000;
    double dist = 1; // default distance to travel
    const double maxDist = 1;
    double stepSize = 0.1;
    double safeDist = 0.05;
    // Initialize matrices
    q_L.resize(1,2);
    q_H.resize(1,2);
    Q.resize(1,2);
    QBoundary.resize(1,2);

    q_L.row(0) = q.transpose();
    q_H.row(0) = q.transpose();
    Q.row(0) = q.transpose();

    path.waypoints.push_back(q);

    collision coll;
    int maxSteps = 10000;
    int steps = 0;
    goalReached = false;
    collide = false;
    boundaryFollowing = false;


    do{
        
        Eigen::Vector2d dir = (q_goal - q)/((q_goal - q).norm()); // direction to goal
        Eigen::Vector2d dist2Goal = q_goal - q; // distance to goal
        if(dist2Goal.norm() <= tol){
            // goal reached
            std::cout << "Goal Reached" << std::endl;
            goalReached = true;
            path.waypoints.push_back(q_goal);
            return path;
        }
        
        q_next = q + dist * dir; // create test point to move to

        collision::Result result = coll.collisionCheckAll(q_next, problem);

        // check if at goal
        if((q_goal - q_next).norm() <= tol) {
            // next point is goal
            std::cout << "Next point is goal" << std::endl;
            goalReached = true;
            path.waypoints.push_back(q_next);
            path.waypoints.push_back(q_goal);
            return path;
        }
        else if(result.hit){
            dist -= 0.01;
            if(dist <= safeDist){
                // we have a collision
                std::cout << "Collision found" << std::endl;
                collide = true;
                // reset dist
                dist = maxDist;

                // push back to this point
                path.waypoints.push_back(q_next);
                q = q_next;
                // add this point as a hit point
                q_H.row(i) = q;
                boundaryFollowing = true;





                while(boundaryFollowing) {
                    // do 8 direction check
                    double checkDist = 0.1; // might have to reduce this for better fidelity
                    std::vector<int> validDirs;

                    // 8 neighbors around q
                    std::vector<Eigen::Vector2d> neighbors = {
                        Eigen::Vector2d(0, 1),   // 0: North
                        Eigen::Vector2d(1, 1).normalized(),   // 1: NorthEast
                        Eigen::Vector2d(1, 0),   // 2: East
                        Eigen::Vector2d(1, -1).normalized(),  // 3: SouthEast
                        Eigen::Vector2d(0, -1),  // 4: South
                        Eigen::Vector2d(-1, -1).normalized(), // 5: SouthWest
                        Eigen::Vector2d(-1, 0),  // 6: West
                        Eigen::Vector2d(-1, 1).normalized()   // 7: NorthWest
                    };

                    // initialize current direction of travel
                    Eigen::Vector2d moveDir = (q_goal - q).normalized();
                    int currentDir = 0;
                    double bestDot = -1e9;
                    for(int jj = 0; jj<8; jj++) {
                        double dot = moveDir.dot(neighbors[jj]);
                        if(dot > bestDot) {
                            bestDot = dot;
                            currentDir = jj;
                        }
                    }



                    for(int ii = 0; ii<8; ii++) {
                        // check direction ii
                        Eigen::Vector2d candidate = q + checkDist * neighbors[ii]; // test point

                        collision::Result dirCheck = coll.collisionCheckAll(candidate, problem); // next to check every obstacle incase of composite obstacles
                        // store collision to find valid directions to move
                        if(!dirCheck.hit) {
                            validDirs.push_back(ii);
                        }
                    }

                    int nextDir = currentDir;

                    // move CCW or CW based on that check
                    if(direction == 'L') {
                        // CCW
                        nextDir = (currentDir + 1) % 8; 
                    } else {
                        // CW
                        nextDir = (currentDir - 1 + 8) % 8;
                    }
                    // record q_L dist, update if needed
                    Eigen::Vector2d q_boundary = q + checkDist * neighbors[nextDir];

                    path.waypoints.push_back(q_boundary);
                    q = q_boundary;
                    

                    // repeat

                    // once q_H re-encountered, keep moving

                    // once best q_L found

                    // stop boundary following
                }
                


            }  
        }
        else{
            // not at goal nor colliding, good to move
            Eigen::Vector2d q_next_valid = q + dist*dir;
            path.waypoints.push_back(q_next_valid);
            q = q_next_valid;
        }
        count++;

    }while((!goalReached && count <= maxCount));

    return path;
}


