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