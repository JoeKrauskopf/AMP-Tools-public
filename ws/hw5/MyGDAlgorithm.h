#pragma once

// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

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
		MyPotentialFunction(
        const Eigen::Vector2d& goal,
        const amp::Problem2D& problem,
        double d_star_, double zetta_
    	) : q_goal(goal), prob(problem), d_star(d_star_), zetta(zetta_) {}

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
			// find number of obstacles

			// for i = 1:n 
				// calculate distance d(i) from point q to obstacle Oi
					// d(i) is the smallest d(q,c) for all points c in QO
					// need to find a way to grab the closest point on obstacle i to point q
						// find 2 closest vertices, move along line testing canidate points between vertices
				// calc Urepi
			
			// Urep = sum(Urepi)
			


			// placeholder for now
			double U_rep = 0;

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

			// repulsive force gradient
			Eigen::Vector2d deltaU_rep;
			// implement similar method to the potential calculation
			// placeholder for now
			deltaU_rep.x() = 0;
			deltaU_rep.y() = 0;

			// combined gradient

			Eigen::Vector2d deltaU = deltaU_att + deltaU_rep;

            return Eigen::Vector2d(deltaU);
        }
	private:
		Eigen::Vector2d q_goal;
		amp::Problem2D prob;
		double d_star;
    	double zetta;
};