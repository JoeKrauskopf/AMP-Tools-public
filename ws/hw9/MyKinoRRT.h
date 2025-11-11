#pragma once

// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"
#include <vector>
#include <iostream>

// Include the correct homework headers
#include "hw/HW9.h"

class MyKinoRRT : public amp::KinodynamicRRT {
    public:
        virtual amp::KinoPath plan(const amp::KinodynamicProblem2D& problem, amp::DynamicAgent& agent) override;
        int max_iterations = 200000;
        int max_attempts = 1;
        double step_size = 1;
        double epsilon = 0.5; // goal tolerance
        double goalBias = 0.05; // goal bias
        int max_controls = 10;
        const double dt = 0.001;
        
        struct RRTEdge {
            Eigen::VectorXd state;
            Eigen::VectorXd control;
            double duration;
        };
        struct TrajectoryStep {
            Eigen::VectorXd state;
            Eigen::VectorXd control;
            double dt;
        };
};  

class angleWrap {
    public: 
        static double wrap(double theta) {
            // wraps angle to -pi, pi
            while (theta > M_PI) theta -= 2.0 * M_PI;
            while (theta < -M_PI) theta += 2.0 * M_PI;
            return theta;
        };
};

class MySingleIntegrator : public amp::DynamicAgent {
    public:
        virtual void propagate(Eigen::VectorXd& state, Eigen::VectorXd& control, double dt) override;
        double U1Max = 3;
        double U1Min = -1.5;
        double U2Max = 3;
        double U2Min = -1.5;
};

class MyFirstOrderUnicycle : public amp::DynamicAgent {
    public:
        virtual void propagate(Eigen::VectorXd& state, Eigen::VectorXd& control, double dt) override;
        double r = 0.25; // radius
        bool simple = false;
        double thetaMax = M_PI;
        double thetaMin = -M_PI;
        double U1Max = 4;
        double U1Min = -2;
        double U2Max = 1.5;
        double U2Min = -1.5;

        
};

class MySecondOrderUnicycle : public amp::DynamicAgent {
    public:
        virtual void propagate(Eigen::VectorXd& state, Eigen::VectorXd& control, double dt) override;
        double r = 0.25;
        bool simple = false;
        double thetaMax = M_PI;
        double thetaMin = -M_PI;
        double sigmaMax = 4;
        double sigmaMin = -3;
        double omegaMax = 1.5;
        double omegaMin = -1.5;
        double U1Max = 1.5;
        double U1Min = -1;
        double U2Max = 0.75;
        double U2Min = -0.75;

};

class MySimpleCar : public amp::DynamicAgent {
    public:
        virtual void propagate(Eigen::VectorXd& state, Eigen::VectorXd& control, double dt) override;
        //double L = 5;
        //double W = 2;
        bool simple = true;
};