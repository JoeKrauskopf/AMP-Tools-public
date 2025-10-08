#include "ManipulatorSkeleton.h"
#include <iostream> // make sure this is included at the top

/*
MyManipulator2D::MyManipulator2D()
    : LinkManipulator2D({1, 0.5, 1}) 
{}
*/



MyManipulator2D::MyManipulator2D()
    : LinkManipulator2D({1, 1}) 
{}


// Override this method for implementing forward kinematics
Eigen::Vector2d MyManipulator2D::getJointLocation(const amp::ManipulatorState& state, uint32_t joint_index) const {
    // Implement forward kinematics to calculate the joint position given the manipulator state (angles)

    const std::vector<double>& linkLengths = getLinkLengths(); //get link lengths
    const Eigen::Vector2d& baseLoc =  getBaseLocation(); // get base position
    int n = state.size(); // number of thetas

    /*
    // Print base location
    std::cout << "Base location: " << baseLoc.transpose() << std::endl;

    // Print link lengths
    std::cout << "Link lengths: ";
    for (size_t i = 0; i < linkLengths.size(); ++i) {
        std::cout << linkLengths[i] << " ";
    }
    std::cout << std::endl;

    // Print joint angles (state)
    std::cout << "Joint angles (state): ";
    for (int i = 0; i < n; ++i) {
        std::cout << state[i] << " ";
    }
    std::cout << std::endl;
    */

    double angleSum = 0;

    std::vector<Eigen::Vector2d> joint_positions;
    Eigen::Vector2d pos = baseLoc;
    joint_positions.push_back(pos);

    for(int i = 0; i<n; i++) {
        // grab angle
        double theta = state[i];
        angleSum += theta;
        // grab link length
        double a = linkLengths[i];
        // calculate x and y pos
        pos.x() += a*cos(angleSum);
        pos.y() += a*sin(angleSum);
        joint_positions.push_back(pos);
 
    }
    //std::reverse(joint_positions.begin(), joint_positions.end());


    //std::vector<Eigen::Vector2d> joint_positions = {Eigen::Vector2d(0.0, 0.0), Eigen::Vector2d(0.0, 1.0), Eigen::Vector2d(1.0, 1.0), Eigen::Vector2d(2.0,1.0)};

    return joint_positions[joint_index];
}

// Override this method for implementing inverse kinematics
amp::ManipulatorState MyManipulator2D::getConfigurationFromIK(const Eigen::Vector2d& end_effector_location) const {
    // Implement inverse kinematics here
    const std::vector<double>& linkLengths = getLinkLengths(); //get link lengths;

    //std::cout <<"Num Links: " << nLinks() << std::endl;

    // Print link lengths
    /*
    std::cout << "Link lengths: ";
    for (size_t i = 0; i < linkLengths.size(); ++i) {
        std::cout << linkLengths[i] << " ";
    }
    std::cout << std::endl;
    */
    amp::ManipulatorState joint_angles(nLinks());
    joint_angles.setZero();
    //std::cout << "Joint angles: " << joint_angles.transpose() << " Num Joint angles: " << joint_angles.size() << std::endl;

    double x = end_effector_location.x();
    double y = end_effector_location.y();
    //std::cout <<"End Effector Location: " << end_effector_location.transpose() << std::endl;
    double distFromGoal;
    Eigen::Vector2d joint3Pos;
    double bestError;
    amp::ManipulatorState bestAngles;
    // If you have different implementations for 2/3/n link manipulators, you can separate them here
    if (nLinks() == 2) {
        double a1 = linkLengths[0]; // link length1
        double a2 = linkLengths[1]; // link length2

        double c2 = (1/(2*a1*a2)) * ((x*x + y*x) - (a1*a1 + a2*a1));
        c2 = std::max(-1.0, std::min(1.0, c2)); // clamp
        double s2Plus   = sqrt(1-c2*c2);
        double s2Minus  = -sqrt(1-c2*c2);
        double c1Plus   = (1/(x*x + y*y)) * (x*(a1 + a2*c2) + y*a2*sqrt(1-c2*c2));
        double c1Minus  = (1/(x*x + y*y)) * (x*(a1 + a2*c2) - y*a2*sqrt(1-c2*c2));
        double s1Plus   = (1/(x*x + y*y)) * (y*(a1 + a2*c2) + x*a2*sqrt(1-c2*c2));
        double s1Minus  = (1/(x*x + y*y)) * (y*(a1 + a2*c2) - x*a2*sqrt(1-c2*c2));

        // prob need to play around with this
        double theta2a = atan2(s2Minus, c2);
        double theta2b = atan2(s2Plus, c2);

        double theta1 = atan2(s1Minus,c1Minus);

        // change for later or nah to do what im doing in 3-link // need to implement
        // now need to check every permutation of thetas to find if any result in a combination
    std::vector<double> theta1_candidates = {theta1};
    std::vector<double> theta2_candidates = {theta2a, theta2b};

    bestError = std::numeric_limits<double>::max();
    bestAngles = joint_angles;
    for (double t1 : theta1_candidates) {
        for (double t2 : theta2_candidates) {
            joint_angles(0) = t1;
            joint_angles(1) = t2;


            Eigen::Vector2d joint2Pos = getJointLocation(joint_angles, 2);
            double error = (end_effector_location - joint2Pos).norm();

            if (error < bestError) {
                bestError = error;
                bestAngles = joint_angles;
            }

            // Optional: if error is already very small, break early
            if (error <= 1e-6) {
                std::cout << "Found exact solution: " << joint2Pos.transpose() << std::endl;
                return joint_angles;
            }
        }
    }
        
        return joint_angles;
    } else if (nLinks() == 3) {

        // need to fix theta 1
        double a1 = linkLengths[0];
        //double theta1 = M_PI/4; // fix at 45 deg
        for(double theta1 = 0; theta1 < 2*M_PI; theta1 += 0.01) {

            //std::cout << "Fixed theta 1 to: " << theta1 << std::endl;
            // then recalc new end point in local reference frame
        
            double x_rel = x - a1 * cos(theta1);
            double y_rel = y - a1 * sin(theta1);
            double x_prime =  x_rel * cos(theta1) + y_rel * sin(theta1);
            double y_prime = -x_rel * sin(theta1) + y_rel * cos(theta1);
            
            //std::cout << "x_prime: " << x_prime << " y_prime: " << y_prime << std::endl;
            // then just use 2 link kinematics to calc theta 2,3
            double a2 = linkLengths[1];
            double a3 = linkLengths[2];

            double c3 = (1/(2*a2*a3)) * ((x_prime*x_prime + y_prime*y_prime) - (a2*a2 + a3*a3));
            c3 = std::max(-1.0, std::min(1.0, c3)); // clamp
            double s3Plus   = sqrt(1-c3*c3);
            double s3Minus  = -sqrt(1-c3*c3);
            double c2Plus   = (1/(x_prime*x_prime + y_prime*y_prime)) * (x_prime*(a2 + a3*c3) + y_prime*a3*sqrt(1-c3*c3));
            double c2Minus  = (1/(x_prime*x_prime + y_prime*y_prime)) * (x_prime*(a2 + a3*c3) - y_prime*a3*sqrt(1-c3*c3));
            double s2Plus   = (1/(x_prime*x_prime + y_prime*y_prime)) * (y_prime*(a2 + a3*c3) + x_prime*a3*sqrt(1-c3*c3));
            double s2Minus  = (1/(x_prime*x_prime + y_prime*y_prime)) * (y_prime*(a2 + a3*c3) - x_prime*a3*sqrt(1-c3*c3));

            /*
            std::cout << "c3: " << c3 << std::endl;
            std::cout << "s3Plus: " << s3Plus << " | s3Minus: " << s3Minus << std::endl;

            std::cout << "c2Plus: " << c2Plus << " | c2Minus: " << c2Minus << std::endl;
            std::cout << "s2Plus: " << s2Plus << " | s2Minus: " << s2Minus << std::endl;
            */
            // prob need to play around with this
            double theta3a = atan2(s3Minus,c3); 
            double theta3b = atan2(s3Plus, c3);
            double theta2a = atan2(s2Minus,c2Minus);
            double theta2b = atan2(s2Plus,c2Plus);
            double theta2c = atan2(s2Minus,c2Plus);
            double theta2d = atan2(s2Plus,c2Minus);

            // now need to check every permutation of thetas to find if any result in a combination
            std::vector<double> theta2_candidates = {theta2a, theta2b, theta2c, theta2d};
            std::vector<double> theta3_candidates = {theta3a, theta3b};

            bestError = std::numeric_limits<double>::max();
            bestAngles = joint_angles;
            for (double t2 : theta2_candidates) {
                for (double t3 : theta3_candidates) {
                    joint_angles(0) = theta1;
                    joint_angles(1) = t2;
                    joint_angles(2) = t3;

                    Eigen::Vector2d joint3Pos = getJointLocation(joint_angles, 3);
                    double error = (end_effector_location - joint3Pos).norm();

                    if (error < bestError) {
                        bestError = error;
                        bestAngles = joint_angles;
                    }

                    // Optional: if error is already very small, break early
                    if (error <= 1e-6) {
                        std::cout << "Found exact solution: " << joint3Pos.transpose() << std::endl;
                        return joint_angles;
                    }
                }
            }
        }
        // if no exact match, return best found
        std::cout << "Best error: " << bestError << std::endl;
        std::cout << "Dist from goal: " << distFromGoal << std::endl;
        std::cout << "Joint3 Pos: " << joint3Pos.transpose() << std::endl;
        return bestAngles;
    } else {
        // no clue

        return joint_angles;
    }

    return joint_angles;
}
