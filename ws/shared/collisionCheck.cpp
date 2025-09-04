#include "collisionCheck.h"

// Need to test!!!

bool collision::collisionCheck(const Eigen::Vector2d q, const amp::Polygon& obstacle) {
    const auto& vertices = obstacle.verticesCCW();
    int n = static_cast<int>(vertices.size());
    double xq = q(0); // point X
    double yq = q(1); // point Y
    int crossings = 0;

    for (int i = 0; i < n; ++i) {
        const Eigen::Vector2d& v1 = vertices[i];
        const Eigen::Vector2d& v2 = vertices[(i + 1) % n]; // wrap around

        // ---- Boundary check ----
        double lineCheck = (xq - v1.x()) * (v2.y() - v1.y()) - (yq - v1.y()) * (v2.x() - v1.x());
        if (std::abs(lineCheck) < 1e-12 &&
            std::min(v1.x(), v2.x()) <= xq && xq <= std::max(v1.x(), v2.x()) &&
            std::min(v1.y(), v2.y()) <= yq && yq <= std::max(v1.y(), v2.y())) {
            collisionDetected = true;
            return collisionDetected; // must return a bool
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
    return collisionDetected;
}

bool collision::collisionCheckAll(const Eigen::Vector2d q, const amp::Problem2D& problem) {
    int n = problem.obstacles.size(); // number of obstacles in the workspace
    collision coll;
    for(int j = 0; j<n; j++)
    {
        const amp::Polygon& obst = problem.obstacles[j];
        if(coll.collisionCheck(q, obst))
        {
            collisionDetected = true;
            return collisionDetected;
            break;
        }
    }
    // If no collision found
    collisionDetected = false;
    return collisionDetected;
}