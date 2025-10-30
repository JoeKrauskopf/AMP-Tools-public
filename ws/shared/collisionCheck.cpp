#include "collisionCheck.h"

// Need to test!!!

collision::Check collision::collisionCheck(const Eigen::Vector2d& q, const amp::Polygon& obstacle) {
    const auto& vertices = obstacle.verticesCCW();
    int n = static_cast<int>(vertices.size());
    double xq = q(0); // point X
    double yq = q(1); // point Y
    int crossings = 0;
    int edgeIdx = -1;

    for (int i = 0; i < n; ++i) {
        const Eigen::Vector2d& v1 = vertices[i];
        const Eigen::Vector2d& v2 = vertices[(i + 1) % n]; // wrap around

        // ---- Boundary check ----
        double lineCheck = (xq - v1.x()) * (v2.y() - v1.y()) - (yq - v1.y()) * (v2.x() - v1.x());
        if (std::abs(lineCheck) < 1e-12 &&
            std::min(v1.x(), v2.x()) <= xq && xq <= std::max(v1.x(), v2.x()) &&
            std::min(v1.y(), v2.y()) <= yq && yq <= std::max(v1.y(), v2.y())) {
            collisionDetected = true;
            edgeIdx = i;
            return {collisionDetected, edgeIdx}; // must return a bool
        }


        // ---- Ray casting ----
        bool condY = (v1.y() > yq) != (v2.y() > yq);
        if (condY) {
            double xIntersection = v1.x() + (yq - v1.y()) * (v2.x() - v1.x()) / (v2.y() - v1.y());
            if (xIntersection > xq) {
                crossings++;
            }
        }
    }

    // Odd number of crossings → inside polygon
    collisionDetected = (crossings % 2 == 1);
    return {collisionDetected, edgeIdx};
}

collision::Result collision::collisionCheckAll(const Eigen::Vector2d& q, const amp::Problem2D& problem) {
    int n = problem.obstacles.size(); // number of obstacles in the workspace
    collision coll;
    for(int j = 0; j<n; j++)
    {
        const amp::Polygon& obst = problem.obstacles[j];
        collision::Check check = coll.collisionCheck(q, obst);
        if(check.hit)
        {
            return {check.hit, j, check.edgeHit};
            break;
        }
    }
    // If no collision found
    collisionDetected = false;
    return {collisionDetected, -1, -1};
}

collision::Result collision::collisionCheckAllEnv(const Eigen::Vector2d& q, const amp::Environment2D& env) {
    int n = env.obstacles.size(); // number of obstacles in the workspace
    for (int j = 0; j < n; j++) {
        const amp::Polygon& obst = env.obstacles[j];
        collision::Check check = collisionCheck(q, obst);
        if (check.hit) {
            return {true, j, check.edgeHit};
        }
    }
    return {false, -1, -1};
}

collision::Check collision::collisionCheckDiskAgent(const Eigen::Vector2d& q, const double radius, const amp::Polygon& obstacle) {
    // NEED TO CHANGE FOR COLLISION WITH RADIUS
    const auto& vertices = obstacle.verticesCCW();
    int n = static_cast<int>(vertices.size());
    double xq = q(0); // point X
    double yq = q(1); // point Y
    int crossings = 0;
    int edgeIdx = -1;

    for (int i = 0; i < n; ++i) {
        const Eigen::Vector2d& v1 = vertices[i];
        const Eigen::Vector2d& v2 = vertices[(i + 1) % n];

        // Compute projection of q onto the edge (v1-v2)
        Eigen::Vector2d edge = v2 - v1;
        Eigen::Vector2d v1ToQ = q - v1;
        double t = v1ToQ.dot(edge) / edge.squaredNorm();
        t = std::clamp(t, 0.0, 1.0);

        // Closest point on edge
        Eigen::Vector2d closest = v1 + t * edge;

        // Distance from q to edge
        double dist = (q - closest).norm();
        if (dist <= radius + 1e-3) {
            collisionDetected = true;
            edgeIdx = i;
            return {collisionDetected, edgeIdx};
        }
    }


        // ---- Ray casting ----
    for (int i = 0; i < n; ++i) {
        const Eigen::Vector2d& v1 = vertices[i];
        const Eigen::Vector2d& v2 = vertices[(i + 1) % n];

        bool condY = (v1.y() > yq) != (v2.y() > yq);
        if (condY) {
            double xIntersection = v1.x() + (yq - v1.y()) * (v2.x() - v1.x()) / (v2.y() - v1.y());
            if (xIntersection > xq) {
                crossings++;
            }
        }
    }

    // Odd number of crossings → inside polygon
    collisionDetected = (crossings % 2 == 1);
    return {collisionDetected, edgeIdx};
}

collision::Result collision::collisionCheckDiskAgentAllEnv(const Eigen::Vector2d& q, const double radius, const amp::Environment2D& env) {
    int n = env.obstacles.size(); // number of obstacles in the workspace
    for (int j = 0; j < n; j++) {
        const amp::Polygon& obst = env.obstacles[j];
        collision::Check check = collisionCheckDiskAgent(q, radius, obst);
        if (check.hit) {
            return {true, j, check.edgeHit};
        }
    }
    return {false, -1, -1};
}

collision::Check collision::collisionCheckDynamicAgent(const Eigen::Vector2d& q, const amp::DynamicAgent& agent, const double theta, const amp::Polygon& obstacle, const amp::Environment2D& env) 
    {
    // Pull length and width from dynamic agent
    double L = agent.agent_dim.length;
    double W = agent.agent_dim.width;

    //std::cout << "[COLLISION] agent L and W: " << L << " " << W << std::endl;
    
    // Create the 4 corners of the rectangle centered at origin
    // Assuming the agent's center is at q and it's oriented along theta
    std::vector<Eigen::Vector2d> localCorners = {
        Eigen::Vector2d(0, -W/2),  // back-left
        Eigen::Vector2d(L, -W/2),   // front-left
        Eigen::Vector2d(L, W/2),    // front-right
        Eigen::Vector2d(0, W/2)    // back-right
    };
    
    // Rotation matrix
    double cosTheta = std::cos(theta);
    double sinTheta = std::sin(theta);
    Eigen::Matrix2d R;
    R << cosTheta, -sinTheta,
         sinTheta, cosTheta;
    
    // Transform corners to world coordinates
    std::vector<Eigen::Vector2d> agentCorners;
    for (const auto& corner : localCorners) {
        agentCorners.push_back(R * corner + q);
    }
    
    const auto& obstVertices = obstacle.verticesCCW();
    int nObs = static_cast<int>(obstVertices.size());
    int nAgent = static_cast<int>(agentCorners.size());
    
    // Check 1: Any agent corner inside obstacle
    for (const auto& corner : agentCorners) {
        collision::Check check = collisionCheck(corner, obstacle);
        if (check.hit) {
            return {true, check.edgeHit};
        }
    }
    
    // Check 2: Any obstacle vertex inside agent rectangle
    for (const auto& vertex : obstVertices) {
        // Point-in-polygon test for agent rectangle
        int crossings = 0;
        for (int i = 0; i < nAgent; ++i) {
            const Eigen::Vector2d& v1 = agentCorners[i];
            const Eigen::Vector2d& v2 = agentCorners[(i + 1) % nAgent];
            
            bool condY = (v1.y() > vertex.y()) != (v2.y() > vertex.y());
            if (condY) {
                double xIntersection = v1.x() + (vertex.y() - v1.y()) * (v2.x() - v1.x()) / (v2.y() - v1.y());
                if (xIntersection > vertex.x()) {
                    crossings++;
                }
            }
        }
        if (crossings % 2 == 1) {
            return {true, -1};
        }
    }
    
    // Check 3: Edge-edge intersection (SAT or direct check)
    for (int i = 0; i < nAgent; ++i) {
        const Eigen::Vector2d& a1 = agentCorners[i];
        const Eigen::Vector2d& a2 = agentCorners[(i + 1) % nAgent];
        
        for (int j = 0; j < nObs; ++j) {
            const Eigen::Vector2d& b1 = obstVertices[j];
            const Eigen::Vector2d& b2 = obstVertices[(j + 1) % nObs];
            
            // Check if segments (a1,a2) and (b1,b2) intersect
            Eigen::Vector2d r = a2 - a1;
            Eigen::Vector2d s = b2 - b1;
            Eigen::Vector2d qMinusP = b1 - a1;
            
            double rxs = r.x() * s.y() - r.y() * s.x();
            double qpxr = qMinusP.x() * r.y() - qMinusP.y() * r.x();
            
            if (std::abs(rxs) > 1e-10) {
                double t = (qMinusP.x() * s.y() - qMinusP.y() * s.x()) / rxs;
                double u = qpxr / rxs;
                
                if (t >= 0.0 && t <= 1.0 && u >= 0.0 && u <= 1.0) {
                    return {true, j};
                }
            }
        }
    }
    
    // --- No collision detected ---
    return {false, -1};
}


collision::Result collision::collisionCheckDynamicAgentAllEnv(const Eigen::Vector2d& q, const amp::DynamicAgent& agent, const double theta, const amp::Environment2D& env) {
    int n = env.obstacles.size(); // number of obstacles in the workspace
    for (int j = 0; j < n; j++) {
        const amp::Polygon& obst = env.obstacles[j];
        collision::Check check = collisionCheckDynamicAgent(q, agent, theta, obst, env);
        if (check.hit) {
            return {true, j, check.edgeHit};
        }
    }
    return {false, -1, -1};
}