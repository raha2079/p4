///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 4
// Authors: FILL ME OUT!!
//////////////////////////////////////

#include <iostream>
#include <cmath>
#include <fstream>

#include <ompl/base/ProjectionEvaluator.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

#include <ompl/control/SimpleSetup.h>
#include <ompl/control/ODESolver.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/control/planners/kpiece/KPIECE1.h>

// Your implementation of RG-RRT
#include "RG-RRT.h"

// Your projection for the pendulum
class PendulumProjection : public ompl::base::ProjectionEvaluator
{
public:
    PendulumProjection(const ompl::base::StateSpace *space) : ProjectionEvaluator(space)
    {
    }

    unsigned int getDimension() const override
    {
        // Use 2D projection: (theta, omega)
        // This captures both position and velocity for good coverage estimation
        return 2;
    }

    void project(const ompl::base::State *state, Eigen::Ref<Eigen::VectorXd> projection) const override
    {
        // Project the state to (theta, omega)
        const auto *s = state->as<ompl::base::RealVectorStateSpace::StateType>();
        projection[0] = s->values[0];  // theta
        projection[1] = s->values[1];  // omega
    }
};

void pendulumODE(const ompl::control::ODESolver::StateType &q, const ompl::control::Control *control,
                 ompl::control::ODESolver::StateType &qdot)
{
    // Extract the current state
    // q[0] = theta (angle of pendulum)
    // q[1] = omega (angular velocity)
    const double theta = q[0];
    const double omega = q[1];

    // Extract the control input (torque)
    // We need to cast the control to get the actual torque value
    const double *u = control->as<ompl::control::RealVectorControlSpace::ControlType>()->values;
    const double torque = u[0];

    // Constants
    const double g = 9.81; // gravity

    // Compute the derivatives according to the dynamics:
    // theta_dot = omega
    // omega_dot = -g * cos(theta) + torque
    qdot.resize(q.size(), 0);
    qdot[0] = omega;                      // Rate of change of angle
    qdot[1] = -g * std::cos(theta) + torque;  // Rate of change of velocity
}

ompl::control::SimpleSetupPtr createPendulum(double torque)
{
    // Step 1: Create the state space (2D: theta and omega)
    // theta can be any angle, omega has limits
    auto space = std::make_shared<ompl::base::RealVectorStateSpace>(2);

    // Set bounds for the state space
    ompl::base::RealVectorBounds bounds(2);
    bounds.setLow(-10);   // Allow theta and omega to range widely
    bounds.setHigh(10);

    // Set specific bounds for omega (angular velocity)
    bounds.setLow(1, -10);   // omega minimum
    bounds.setHigh(1, 10);   // omega maximum (as specified in PDF)

    space->setBounds(bounds);

    // Register projection for KPIECE
    space->registerDefaultProjection(std::make_shared<PendulumProjection>(space.get()));

    // Step 2: Create the control space (1D: torque)
    auto cspace = std::make_shared<ompl::control::RealVectorControlSpace>(space, 1);

    // Set bounds for control (torque)
    ompl::base::RealVectorBounds cbounds(1);
    cbounds.setLow(-torque);   // Minimum torque
    cbounds.setHigh(torque);   // Maximum torque
    cspace->setBounds(cbounds);

    // Step 3: Create SimpleSetup
    auto ss = std::make_shared<ompl::control::SimpleSetup>(cspace);

    // Step 4: Set up the ODE solver
    // This will use our pendulumODE function to simulate physics
    auto odeSolver = std::make_shared<ompl::control::ODEBasicSolver<>>(ss->getSpaceInformation(), &pendulumODE);
    ss->setStatePropagator(ompl::control::ODESolver::getStatePropagator(odeSolver));

    // Step 5: Set state validity checker
    // For pendulum, we just need to check omega is within bounds
    ss->setStateValidityChecker([](const ompl::base::State *state) {
        // Cast state to get actual values
        const auto *s = state->as<ompl::base::RealVectorStateSpace::StateType>();
        // Check if omega (velocity) is within bounds [-10, 10]
        double omega = s->values[1];
        return std::abs(omega) <= 10.0;
    });

    // Step 6: Set start and goal states
    // Start: hanging down at rest: theta = -pi/2, omega = 0
    // Goal: pointing up at rest: theta = +pi/2, omega = 0
    ompl::base::ScopedState<> start(space);
    start[0] = -M_PI / 2.0;  // theta: hanging down
    start[1] = 0.0;           // omega: at rest

    ompl::base::ScopedState<> goal(space);
    goal[0] = M_PI / 2.0;    // theta: pointing up
    goal[1] = 0.0;            // omega: at rest

    // Set start and goal with a threshold (we can't hit exactly, so within 0.5 radians is OK)
    ss->setStartAndGoalStates(start, goal, 0.5);

    return ss;
}

void planPendulum(ompl::control::SimpleSetupPtr &ss, int choice)
{
    // Step 1: Create the planner based on user's choice
    ompl::base::PlannerPtr planner;

    if (choice == 1)
    {
        // RRT planner
        planner = std::make_shared<ompl::control::RRT>(ss->getSpaceInformation());
        std::cout << "Using RRT planner" << std::endl;
    }
    else if (choice == 2)
    {
        // KPIECE1 planner
        planner = std::make_shared<ompl::control::KPIECE1>(ss->getSpaceInformation());
        std::cout << "Using KPIECE1 planner" << std::endl;
    }
    else if (choice == 3)
    {
        // RG-RRT planner (TODO: implement for final submission)
        // planner = std::make_shared<ompl::control::RGRRT>(ss->getSpaceInformation());
        std::cout << "RG-RRT not implemented yet. Using RRT instead." << std::endl;
        planner = std::make_shared<ompl::control::RRT>(ss->getSpaceInformation());
    }

    // Step 2: Set the planner
    ss->setPlanner(planner);

    // Step 3: Try to solve the problem within 60 seconds
    std::cout << "Planning..." << std::endl;
    ompl::base::PlannerStatus solved = ss->solve(60.0);

    // Step 4: Check if we found a solution
    if (solved)
    {
        std::cout << "Found solution!" << std::endl;

        // Note: simplifySolution() doesn't exist for control planning
        // The path is already optimized by the planner

        // Get the solution path
        ompl::control::PathControl &path = ss->getSolutionPath();

        // Print path to console
        path.printAsMatrix(std::cout);

        // Save path to file for visualization
        std::ofstream fout("pendulum_path.txt");
        // Convert to geometric path for easier plotting
        path.asGeometric().printAsMatrix(fout);
        fout.close();
        std::cout << "Path saved to pendulum_path.txt" << std::endl;
    }
    else
    {
        std::cout << "No solution found" << std::endl;
    }
}

void benchmarkPendulum(ompl::control::SimpleSetupPtr &/* ss */)
{
    // TODO: Do some benchmarking for the pendulum
}

int main(int /* argc */, char ** /* argv */)
{
    int choice;
    do
    {
        std::cout << "Plan or Benchmark? " << std::endl;
        std::cout << " (1) Plan" << std::endl;
        std::cout << " (2) Benchmark" << std::endl;

        std::cin >> choice;
    } while (choice < 1 || choice > 2);

    int which;
    do
    {
        std::cout << "Torque? " << std::endl;
        std::cout << " (1)  3" << std::endl;
        std::cout << " (2)  5" << std::endl;
        std::cout << " (3) 10" << std::endl;

        std::cin >> which;
    } while (which < 1 || which > 3);

    double torques[] = {3., 5., 10.};
    double torque = torques[which - 1];

    ompl::control::SimpleSetupPtr ss = createPendulum(torque);

    // Planning
    if (choice == 1)
    {
        int planner;
        do
        {
            std::cout << "What Planner? " << std::endl;
            std::cout << " (1) RRT" << std::endl;
            std::cout << " (2) KPIECE1" << std::endl;
            std::cout << " (3) RG-RRT" << std::endl;

            std::cin >> planner;
        } while (planner < 1 || planner > 3);

        planPendulum(ss, planner);
    }
    // Benchmarking
    else if (choice == 2)
        benchmarkPendulum(ss);

    else
        std::cerr << "How did you get here? Invalid choice." << std::endl;

    return 0;
}
