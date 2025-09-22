#include "ManipulatorSkeleton.h"
#include <iostream> // make sure this is included at the top


MyManipulator2D::MyManipulator2D()
    : LinkManipulator2D({0.5, 1.0, 0.5}) 
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
    std::reverse(joint_positions.begin(), joint_positions.end());


    //std::vector<Eigen::Vector2d> joint_positions = {Eigen::Vector2d(0.0, 0.0), Eigen::Vector2d(0.0, 1.0), Eigen::Vector2d(1.0, 1.0), Eigen::Vector2d(2.0,1.0)};

    return joint_positions[joint_index];
}

// Override this method for implementing inverse kinematics
amp::ManipulatorState MyManipulator2D::getConfigurationFromIK(const Eigen::Vector2d& end_effector_location) const {
    // Implement inverse kinematics here
    const std::vector<double>& linkLengths = getLinkLengths(); //get link lengths;
    amp::ManipulatorState joint_angles;
    joint_angles.setZero();
    x = end_effector_location.x();
    y = end_effector_location.y();
    
    // If you have different implementations for 2/3/n link manipulators, you can separate them here
    if (nLinks() == 2) {
        double a1 = linkLengths[0]; // link length1
        double a2 = linkLengths[1]; // link length2

        double c2 = (1/(2*a1*a2)) * ((x^2 + y^2) - (a1^2 + a2^2));
        double s2Plus   = sqrt(1-c2^2);
        double s2Minus  = -sqrt(1-c2^2);
        double c1Plus   = (1/(x^2 + y^2)) * (x*(a1 + a2*c2) + y*a2*sqrt(1-c2^2));
        double c1Minus  = (1/(x^2 + y^2)) * (x*(a1 + a2*c2) - y*a2*sqrt(1-c2^2));
        double s1Plus   = (1/(x^2 + y^2)) * (y*(a1 + a2*c2) + x*a2*sqrt(1-c2^2));
        double s1Minus  = (1/(x^2 + y^2)) * (y*(a1 + a2*c2) - x*a2*sqrt(1-c2^2));

        // prob need to play around with this
        double theta1 = acos(c1Minus); 
        double theta2 = acos(c2);


        return joint_angles;
    } else if (nLinks() == 3) {

        return joint_angles;
    } else {

        return joint_angles;
    }

    return joint_angles;
}