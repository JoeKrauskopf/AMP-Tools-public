// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework header
#include "hw/HW5.h"
#include "hw/HW2.h"
// Include any custom headers you created in your workspace
#include "MyGDAlgorithm.h"

using namespace amp;

int main(int argc, char** argv) {
    /* Include this line to have different randomized environments every time you run your code (NOTE: this has no affect on grade()) */
    //amp::RNG::seed(amp::RNG::randiUnbounded());

    // Test your gradient descent algorithm on a random problem.
    double d_star = 1; // threshold distance from goal
    double zetta = 1; // weight on U_att
    double Q_star = 1; // threshold to ignore obstacle
    double eta = 1; // weight on U_rep

    // 5 10 5 0.6 works with HW5Workspace1

    MyGDAlgorithm algo(d_star, zetta, Q_star,eta);
    //Path2D path;
    //Problem2D prob;

    //Problem2D prob = HW5::getWorkspace1();
    Problem2D prob = HW2::getWorkspace1();
    //Problem2D prob = HW2::getWorkspace2();


    //bool success = HW5::generateAndCheck(algo, path, prob);
    Path2D path = algo.plan(prob);
    bool success = HW5::check(path, prob);
    Visualizer::makeFigure(prob, path);

    // Visualize your potential function
    Visualizer::makeFigure(MyPotentialFunction(prob.q_goal, prob, d_star, zetta, Q_star, eta), prob, 80);
    // figure out why this looks weird
    // mightve been fixed see line 453 in visualizer.cpp, check with peter tho
    
    Visualizer::saveFigures();
    
    // Arguments following argv correspond to the constructor arguments of MyGDAlgorithm:
    //HW5::grade<MyGDAlgorithm>("nonhuman.biologic@myspace.edu", argc, argv, d_star, zetta, Q_star, eta);
    return 0;
}