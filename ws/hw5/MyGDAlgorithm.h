#pragma once

// TODO:
// change Q_star to be dynamic (needs to change for each Urep_i)

// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"
#include <iostream>
// Include the correct homework header
#include "hw/HW5.h"

class MyGDAlgorithm : public amp::GDAlgorithm {
	public:
		// Consider defining a class constructor to easily tune parameters, for example: 
		MyGDAlgorithm(double d_star, double zetta, double Q_star, double eta) :
			d_star(d_star),
			zetta(zetta),
			Q_star(Q_star),
			eta(eta) {}

		// Override this method to solve a given problem.
		virtual amp::Path2D plan(const amp::Problem2D& problem) override;
	private:
		double d_star, zetta, Q_star, eta;

		Eigen::Vector2d q;
};

class MyPotentialFunction : public amp::PotentialFunction2D {
    public:	
		struct DistInfo {
			Eigen::Vector2d c; // point on obstacle O that minimizes d
			double d; // min distance from q to c
		};

		MyPotentialFunction(
        const Eigen::Vector2d& goal,
        const amp::Problem2D& problem,
        double d_star_, double zetta_,
		double Q_star_, double eta_
    	) : q_goal(goal), prob(problem), d_star(d_star_), zetta(zetta_), Q_star(Q_star_), eta(eta_) {}

		// Returns the potential function value (height) for a given 2D point. 
        virtual double operator()(const Eigen::Vector2d& q) const override {
			// attractive potential
			double U_att;
			if ((q-q_goal).norm() <= d_star) {
				U_att = 0.5*zetta*((q-q_goal).norm() * (q-q_goal).norm());
			} else {
				U_att = d_star * zetta * (q-q_goal).norm() - 0.5*zetta*(d_star*d_star);
			}
			// repulsive potential
			double U_rep = 0;
			double U_rep_i;
			// find number of obstacles
			int n = prob.obstacles.size(); // number of obstacles in the workspace
			for(int i = 0; i<n; i++) {
				const amp::Polygon& obst = prob.obstacles[i];
				DistInfo distInfo = getD(q, obst);
				double di = distInfo.d;
				Eigen::Vector2d ci = distInfo.c;
				if(di <= Q_star) {
					U_rep_i = 0.5 * eta * ((1/di - 1/Q_star)*(1/di - 1/Q_star));
				}else {
					U_rep_i = 0;
				}
				U_rep += U_rep_i;
				
			}
			// should be added
			double U = U_att + U_rep;

            return U;
        }

		virtual Eigen::Vector2d getGradient(const Eigen::Vector2d& q) const override {
			// attractive force gradient
			Eigen::Vector2d deltaU_att;
			if ((q - q_goal).norm() <= d_star) {
				deltaU_att = zetta*(q-q_goal);
			} else {
				deltaU_att = (d_star * zetta * (q-q_goal))/((q-q_goal).norm());
			}
			//std::cout << "deltaU_att: " << deltaU_att.transpose() << std::endl;
			// repulsive force gradient
			Eigen::Vector2d deltaU_rep;
			deltaU_rep.x() = 0;
			deltaU_rep.y() = 0;

			Eigen::Vector2d deltaU_rep_i;
			// implement similar method to the potential calculation
			// find number of obstacles
			int n = prob.obstacles.size(); // number of obstacles in the workspace
			for(int i = 0; i<n; i++) {

				
				const amp::Polygon& obst = prob.obstacles[i];
				double ri = estimateObstacleRadius(obst);
				const double kappa = 1.5;     // scale factor for obstacle radius
				double Q_star_i = std::max(Q_star, kappa*ri);
				DistInfo distInfo = getD(q, obst);
				double di = distInfo.d;
				Eigen::Vector2d ci = distInfo.c;
				Eigen::Vector2d deltaDi = (q-ci)/(di);
				if(di <= Q_star_i) {
					deltaU_rep_i = eta * ((1/Q_star_i - 1/di)*(deltaDi/(di*di)));
				}else {	
					deltaU_rep_i.setZero();;
				}
				deltaU_rep += deltaU_rep_i;
			}
			double max_rep = 10;
			if (deltaU_rep.norm() > max_rep) {
				deltaU_rep = deltaU_rep.normalized() * max_rep;
			}
			//std::cout << "deltaU_rep: " << deltaU_rep.transpose() << std::endl;
			// combined gradient
			// need to check adding vs subtracting

			Eigen::Vector2d deltaU = deltaU_att + deltaU_rep;
			double max_grad = 10;
			if (deltaU.norm() > max_grad) {
				deltaU = deltaU.normalized() * max_grad;
			}
			//std::cout << "deltaU: " << deltaU.transpose() << std::endl;
            return Eigen::Vector2d(deltaU);
        }

		DistInfo getD(const Eigen::Vector2d& q, const amp::Polygon& obstacle) const {
			DistInfo info;
    		info.d = std::numeric_limits<double>::infinity(); // initialize distance very large
			const auto& vertices = obstacle.verticesCCW();
			int n = static_cast<int>(vertices.size()); // num of vertices in obstacle
			for(int i = 0; i<n; i++){
				const Eigen::Vector2d& v1 = vertices[i];
        		const Eigen::Vector2d& v2 = vertices[(i + 1) % n]; // wrap around
				Eigen::Vector2d edge = v2-v1;
				double edgeLenSqr = edge.squaredNorm();
				// If v1 == v2 (degenerate edge), skip
        		if (edgeLenSqr == 0.0) continue;
				// Projection of q onto the line (normalized by length)
				double t = (q - v1).dot(edge) / edgeLenSqr;
				t = std::max(0.0, std::min(1.0, t)); // clamp to segment

				// Candidate closest point
				Eigen::Vector2d candidate = v1 + t * edge;
				double dist = (q - candidate).norm();

				if (dist < info.d) {
					info.d = dist;     // update min distance
					info.c = candidate; // update closest point
				}
			}
			return info;
		}

		static double estimateObstacleRadius(const amp::Polygon& obst) {
			const auto& verts = obst.verticesCCW();
			// centroid
			Eigen::Vector2d centroid(0,0);
			for (const auto& v : verts) centroid += v;
			centroid /= static_cast<double>(verts.size());
			double maxr = 0.0;
			for (const auto& v : verts) {
				maxr = std::max(maxr, (v - centroid).norm());
			}
			// if degenerate fallback to bounding-box diag / 2
			return maxr;
		}
	private:
		Eigen::Vector2d q_goal;
		amp::Problem2D prob;
		double d_star;
    	double zetta;
		double Q_star;
		double eta;
		
};