// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework header
#include "hw/HW4.h"

// Include the headers for HW4 code
#include "CSpaceSkeleton.h"
#include "ManipulatorSkeleton.h"

// Include the header of the shared class
#include "HelpfulClass.h"

#include <cmath> // for M_PI

using namespace amp;

int main(int argc, char** argv) {
    /* Include this line to have different randomized environments every time you run your code (NOTE: this has no affect on grade()) */
    amp::RNG::seed(amp::RNG::randiUnbounded());

    MyManipulator2D manipulator;

    // You can visualize your manipulator given an angle state like so:
    
    Eigen::Vector2d goal;
    goal << 2,0; // end effector position
    amp::ManipulatorState test_state = manipulator.getConfigurationFromIK(goal);
    std::cout << "Computed joint angles: ";
    for (int i = 0; i < test_state.size(); ++i) {
        std::cout << test_state(i) << " ";
    }
    std::cout << std::endl;
    

    /*
    amp::ManipulatorState test_state;
    test_state.setZero();
    test_state.resize(3);           // make sure it has 3 elements
    test_state[0] = M_PI / 6;       // 45 degrees
    test_state[1] = M_PI / 3;
    test_state[2] = 7*M_PI / 4;
    */
    //test_state[3] = 0;




    
    // The visualizer uses your implementation of forward kinematics to show the joint positions so you can use that to test your FK algorithm
    Visualizer::makeFigure(manipulator, test_state); 
    
    // Create the collision space constructor
    std::size_t n_cells = 200;
    MyManipulatorCSConstructor cspace_constructor(n_cells);

    // Create the collision space using a given manipulator and environment
    std::unique_ptr<amp::GridCSpace2D> cspace = cspace_constructor.construct(manipulator, HW4::getEx3Workspace3());

    // You can visualize your cspace 
    Visualizer::makeFigure(*cspace);
    Visualizer::makeFigure(HW4::getEx3Workspace3());
    Visualizer::saveFigures();

    // Grade method
    //amp::HW4::grade<MyManipulator2D>(cspace_constructor, "joseph.krauskopf@colorado.edu", argc, argv);
    return 0;
}