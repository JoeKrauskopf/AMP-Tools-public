#include "MyBugAlgorithm.h"

// Implement your methods in the `.cpp` file, for example:
amp::Path2D MyBugAlgorithm::plan(const amp::Problem2D& problem) {

    // Your algorithm solves the problem and generates a path. Here is a hard-coded to path for now...
    amp::Path2D path;

    int i = 1;
    float tolerance = 1e-6;

    Eigen::Vector2d q = problem.q_init; // start point
    Eigen::Vector2d mLine = problem.q_goal-problem.q_init;
    path.waypoints.push_back(q);
    q_L(0) = q;

    while(1){
        while(followingObstacle == false){
            
            if((q-q_goal).norm() <= tolerance )
            {
                break; // goal reached
            }
            else if(obstacleHit) // need to implement collision detection
            {
                followingObstacle = true;
            }

        }

    }




    path.waypoints.push_back(problem.q_init);
    path.waypoints.push_back(Eigen::Vector2d(2.0, 5.0));
    path.waypoints.push_back(Eigen::Vector2d(3.0, 9.0));
    path.waypoints.push_back(problem.q_goal);

    return path;
}
