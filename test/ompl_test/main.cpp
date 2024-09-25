// File: ompl_rrt_with_rai.cpp
#include <functional>

#include <Kin/kin.h>                     // RAI's Configuration class
#include <Kin/frame.h>
#include <Gui/opengl.h>                  // For visualization
#include <Kin/viewer.h>

#include <ompl/base/SpaceInformation.h>  // OMPL core components
#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;

int main(int argc, char** argv) {
    // Step 1: Load the RAI configuration
    rai::Configuration C;
    C.addFile("../../rai-robotModels/scenarios/mobileMini.g");  // Load your robot model or environment

    // Optionally, visualize the configuration
    rai::ConfigurationViewer viewer;
    viewer.setConfiguration(C);

    // Step 2: Define start and goal configurations using RAI
    arr q_start = C.getJointState();  // Get current joint state as start
    arr q_goal = q_start;             // Define a goal state (e.g., move a joint)
    q_goal(0) += 0.5;
    q_goal(1) += 0.5;  // Modify joint 0 for demonstration

    // Step 3: Define the OMPL state space matching RAI's configuration space
    unsigned int space_dim = q_start.N;
    auto space = std::make_shared<ob::RealVectorStateSpace>(space_dim);

    // Step 4: Set bounds for each dimension (joint limits)
    ob::RealVectorBounds bounds(space_dim);
    arr limits = C.getJointLimits();  // Get joint limits from RAI

  // Get joint limits from RAI
    for (unsigned int i = 0; i < space_dim; ++i) {
        bounds.setLow(i, limits(0, i));
        bounds.setHigh(i, limits(1, i));
    }
    space->setBounds(bounds);

    // Step 5: Create Space Information
    auto si = std::make_shared<ob::SpaceInformation>(space);

    // Step 6: Define State Validity Checker using RAI's collision checking
    std::function<bool(const ob::State*)> isStateValid = [&](const ob::State* state) {
        const double* values = state->as<ob::RealVectorStateSpace::StateType>()->values;
        arr q(space_dim);
        for (unsigned int i = 0; i < space_dim; ++i) {
            q(i) = values[i];
        }

        // Set the robot's configuration to q
        C.setJointState(q);

        // Check for collisions
        C.ensure_proxies();
        return C.proxies.d0;  // True if no collisions
    };

si->setStateValidityChecker(isStateValid);


    // Optional: Set state validity checking resolution (interpolation between states)
    si->setStateValidityCheckingResolution(0.01);

    // Step 7: Define start and goal states
    ob::ScopedState<> start(space);
    ob::ScopedState<> goal(space);
    for (unsigned int i = 0; i < space_dim; ++i) {
        start[i] = q_start(i);
        goal[i] = q_goal(i);
    }

    // Step 8: Create Problem Definition
    auto pdef = std::make_shared<ob::ProblemDefinition>(si);
    pdef->setStartAndGoalStates(start, goal);

    // Step 9: Select and configure the planner
    auto planner = std::make_shared<og::RRTConnect>(si);
    planner->setProblemDefinition(pdef);
    planner->setup();

    // Optional: Adjust planner parameters
    planner->as<og::RRTConnect>()->setRange(0.1);

    // Step 10: Plan a path
    // Create a termination condition with a 5-second time limit
    ob::PlannerTerminationCondition ptc = ob::timedPlannerTerminationCondition(5.0);

    // Run the planner with the termination condition
    ob::PlannerStatus solved = planner->solve(ptc);


    if (solved) {
        std::cout << "Found solution:" << std::endl;

        // Step 11: Retrieve and process the solution path
        auto path = std::static_pointer_cast<og::PathGeometric>(pdef->getSolutionPath());
        path->printAsMatrix(std::cout);

        // Convert OMPL path to RAI's arr format
        std::vector<ob::State*> states = path->getStates();
        arr trajectory(states.size(), space_dim);
        for (size_t i = 0; i < states.size(); ++i) {
            const double* values = states[i]->as<ob::RealVectorStateSpace::StateType>()->values;
            for (unsigned int j = 0; j < space_dim; ++j) {
                trajectory(i, j) = values[j];
            }
        }

        // Step 12: Visualize the trajectory using RAI
        for (size_t t = 0; t < trajectory.d0; ++t) {
            C.setJointState(trajectory[t]);
            viewer.setConfiguration(C);
            rai::wait(0.1);  // Pause for visualization
        }
    } else {
        std::cout << "No solution found." << std::endl;
    }

    return 0;
}