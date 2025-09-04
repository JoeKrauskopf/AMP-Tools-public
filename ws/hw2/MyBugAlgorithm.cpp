#include "MyBugAlgorithm.h"
#include "collisionCheck.h"
#include <iostream> 

// Implement your methods in the `.cpp` file, for example:
amp::Path2D MyBugAlgorithm::plan(const amp::Problem2D& problem) {

    // Your algorithm solves the problem and generates a path. Here is a hard-coded to path for now...
    amp::Path2D path;

    // Initialize with zero rows, 2 columns
    q_L.resize(0, 2);
    q_H.resize(0, 2);
    Q.resize(0,2);
    collision coll;
   
    

    int i = 1;
    double dist = 1; // move 1 unit at a time
    const double ogDist = 1; // static dist
    double tolerance = 1e-6; // tolerance for finding the goal
    double distTolerance = 0.01; // minimum distance willing to travel
    Eigen::Vector2d q = problem.q_init; // start point
    Eigen::Vector2d q_goal = problem.q_goal; // end point
    Eigen::Vector2d dir; // create direction unit vector to goal line
    Eigen::Vector2d vertDir; // direction vector to next vertex (used for boundary following)
    Eigen::Vector2d q_next;
    
    const amp::Polygon* hitObst = nullptr;

    // need to check if new point is colliding or not

    // -----------------bug 1 algo start--------------------
    // Initialize q_L with the start point
    q_L.resize(1, 2);
    q_L.row(0) = problem.q_init.transpose();
    Q.resize(1,2);
    Q.row(0) = problem.q_init.transpose();
    q = Q.row(0).transpose();
    path.waypoints.push_back(problem.q_init);
    followingObstacle = false;
    exit = false;

    while(!exit)
    {
        // grab last current position
        q = Q.row(i-1).transpose();
        dir = (q_goal - q) / (q_goal - q).norm();   // direction of goal (unit vector)
        q_next = q + dist*dir; // next point to move to

        // check if at goal
        if((q_next - q_goal).norm() <= tolerance)
        {
            std::cout << "Reached goal" << std::endl;
            exit = true;
            path.waypoints.push_back(q_next);
            path.waypoints.push_back(q_goal);
            return path;
        }

        // check if collision
        bool hit = coll:collisionCheckAll(q_next, problem);
        if(!hit)
        {
            // not a collision, were good to move
            move = true;

        }
        else
        {
            // we have a collision
            dist = dist - 0.01;
            if(dist <= distTolerance);
            {
                // reached the edge of an obstacle
                // record hit point
                // engage boundary following

            }
            // while(boundaryFollowing)
                // reset distance
                // find nearest vertex to left or right
                // move to that vertex
                // record distance to goal as q_L, overwrite accordingly
                // break once q_H hit
                // retrace steps to the min q_L
                // turn off boundary following

        }

        // MOVE COMMANDS
        if(move)
        {
            Q.conservativeResize(Q.rows() + 1, 2);
            Q.row(Q.rows() - 1) = q_next.transpose(); // store point into trajectory
            path.waypoints.push_back(q_next);
        }
        

    }

    
    
    
    /*
    while(!exit && i<10)
    {
        q = Q.row(i-1).transpose();
        dir = (q_goal - q) / (q_goal - q).norm();   // direction of goal (unit vector)

        q_next = q + dist*dir; // next point to move to
        
        // check if next point is the goal
        if((q_next - q_goal).norm() <= tolerance)
        {
            std::cout << "Reached goal" << std::endl;
            path.waypoints.push_back(q_next);
            path.waypoints.push_back(q_goal);
            exit = true;
            return path; // goal reached
        }

        // if not:
        // check for collision
        for(int j = 0; j<n; j++)
        {
            const amp::Polygon& obst = problem.obstacles[j];
            if(coll.collisionCheck(q_next, obst))
            {
                std::cout << "Hit Obstacle " << j << std::endl;
                // next point is a collision point
                hitObst = &obst;
                followingObstacle = true;
                //dist = 0.01; // move to very small increments
                break;
            }
        }
        // following obstacle logic
        if(followingObstacle)
        {

            // record as a hit point
            q_H.conservativeResize(q_H.rows() + 1,2);
            q_H.row(q_H.rows()-1) = q.transpose();
            Eigen::Vector2d qHi = q;
            //std::cout << "Hit point " << i << " is " << qHi << std::endl;
            // check if new hit point is same as old hit poiint


            // now move into boundary following
            
            const auto& vertices = hitObst->verticesCCW();
            int closest_idx = 0;

            double min_dist = (q_H.row(q_H.rows() - 1).transpose() - vertices[0]).norm();
            for (size_t i = 1; i < vertices.size(); i++) {
                double dist = (q_H.row(q_H.rows()-1).transpose() - vertices[i]).norm();
                if (dist < min_dist) {
                    min_dist = dist;
                    closest_idx = i;
                }
            }
            Eigen::Vector2d next_vertex;
            // try just moving in direction of next vertex (L or R?)
            if(direction == 'R')
            {
                // right turning
                // find nearest vertex to the right
                int idx = (closest_idx == 0) ? vertices.size() - 1 : closest_idx - 1;
                next_vertex = vertices[idx];
            }   
            else if(direction == 'L')
            {
                // left turning
                //find nearest vertext to the left
                size_t idx = (closest_idx + 1) % vertices.size();
                next_vertex = vertices[idx];
            }
            std::cout << "Hit point: " << qHi << std::endl;
            std::cout << "Nearest vertex on obstacle is " << next_vertex << std::endl;
            vertDir = (next_vertex - qHi) / (next_vertex - qHi).norm();
            double step = std::min(dist, (next_vertex - qHi).norm());   
            q_next = qHi + dist*vertDir;
            // check if new point is colliding with obstacle:
            for(int j = 0; j<n; j++)
            {
                const amp::Polygon& obst = problem.obstacles[j];
                if(!coll.collisionCheck(q_next, obst))
                {
                    followingObstacle = false;
                    break;
                }
                else
                {
                    // idk what to do here
                }
            }
            
        }

        Q.conservativeResize(Q.rows() + 1, 2);
        Q.row(Q.rows() - 1) = q_next.transpose();  // <-- use q_L.rows() not q_H.rows()
        path.waypoints.push_back(q_next);
        i++;
    }
    */



    //path.waypoints.push_back(q_goal);

 

}
