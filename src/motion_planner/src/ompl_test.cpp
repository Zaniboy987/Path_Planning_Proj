#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <iostream>

namespace ob = ompl::base;
namespace og = ompl::geometric;

bool isStateValid(const ob::State *state) {
    return true; // Always valid, for now
}

int main() {
    // Create 2D space
    ob::StateSpacePtr space(new ob::RealVectorStateSpace(2));

    ob::RealVectorBounds bounds(2);
    bounds.setLow(0);
    bounds.setHigh(10);
    space->as<ob::RealVectorStateSpace>()->setBounds(bounds);

    og::SimpleSetup ss(space);
    ss.setStateValidityChecker(isStateValid);

    ob::ScopedState<> start(space);
    start[0] = 1.0;
    start[1] = 1.0;

    ob::ScopedState<> goal(space);
    goal[0] = 9.0;
    goal[1] = 9.0;

    ss.setStartAndGoalStates(start, goal);

    ss.setPlanner(std::make_shared<og::PRM>(ss.getSpaceInformation()));
    ob::PlannerStatus solved = ss.solve(2.0);

    if (solved) {
        std::cout << "Found solution:" << std::endl;
        ss.simplifySolution();
        ss.getSolutionPath().printAsMatrix(std::cout);
    } else {
        std::cout << "No solution found" << std::endl;
    }

    return 0;
}
