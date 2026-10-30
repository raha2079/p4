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
#include <ompl/base/spaces/SE2StateSpace.h>

#include <ompl/control/SimpleSetup.h>
#include <ompl/control/ODESolver.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/control/planners/kpiece/KPIECE1.h>

// The collision checker produced in project 2
#include "CollisionChecking.h"

// Your implementation of RG-RRT
#include "RG-RRT.h"

// Your projection for the car
class CarProjection : public ompl::base::ProjectionEvaluator
{
public:
    CarProjection(const ompl::base::StateSpace *space) : ProjectionEvaluator(space)
    {
    }

    unsigned int getDimension() const override
    {
        // Use 2D projection: (x, y) position
        // This focuses on workspace coverage, which is most important for navigation
        return 2;
    }

    void project(const ompl::base::State *state, Eigen::Ref<Eigen::VectorXd> projection) const override
    {
        // Project the state to (x, y) position
        const auto *s = state->as<ompl::base::RealVectorStateSpace::StateType>();
        projection[0] = s->values[0];  // x position
        projection[1] = s->values[1];  // y position
        // Ignore theta and v for projection - focus on spatial coverage
    }
};

void carODE(const ompl::control::ODESolver::StateType &q, const ompl::control::Control *control,
            ompl::control::ODESolver::StateType &qdot)
{
    // Extract the current state
    // q[0] = x (position)
    // q[1] = y (position)
    // q[2] = theta (heading)
    // q[3] = v (forward velocity)
    const double theta = q[2];
    const double v = q[3];

    // Extract the control input (angular velocity and acceleration)
    const double *u = control->as<ompl::control::RealVectorControlSpace::ControlType>()->values;
    const double omega = u[0];     // Angular velocity
    const double accel = u[1];     // Forward acceleration

    // Compute the derivatives according to the dynamics:
    // x_dot = v * cos(theta)
    // y_dot = v * sin(theta)
    // theta_dot = omega
    // v_dot = accel
    qdot.resize(q.size(), 0);
    qdot[0] = v * std::cos(theta);  // Rate of change of x
    qdot[1] = v * std::sin(theta);  // Rate of change of y
    qdot[2] = omega;                 // Rate of change of heading
    qdot[3] = accel;                 // Rate of change of velocity
}

void makeStreet(std::vector<Rectangle> &obstacles)
{
    // Create a proper street environment with full boundary walls
    // Street bounds: x ∈ [-5, 5], y ∈ [-10, 10]

    // TOP WALL - blocks y >= 6 across FULL width (prevent escape around sides)
    obstacles.push_back(Rectangle{-6, 6, 12, 5});   // x=[-6,6], y=[6,11]

    // BOTTOM WALL - blocks y <= -6 across FULL width
    obstacles.push_back(Rectangle{-6, -11, 12, 5}); // x=[-6,6], y=[-11,-6]

    // LEFT WALL - blocks x <= -5 (keep car in street)
    obstacles.push_back(Rectangle{-6, -6, 1, 12});  // x=[-6,-5], y=[-6,6]

    // RIGHT WALL - blocks x >= 5 (keep car in street)
    obstacles.push_back(Rectangle{5, -6, 1, 12});   // x=[5,6], y=[-6,6]

    // LEFT MIDDLE OBSTACLE - creates maze
    obstacles.push_back(Rectangle{-4, -3, 3, 4});   // x=[-4,-1], y=[-3,1]

    // RIGHT MIDDLE OBSTACLE - creates maze
    obstacles.push_back(Rectangle{1, 2, 3, 3});     // x=[1,4], y=[2,5]
}

ompl::control::SimpleSetupPtr createCar(std::vector<Rectangle> &obstacles, double omega_bound, double accel_bound)
{
    // Step 1: Create the state space (4D: x, y, theta, v)
    auto space = std::make_shared<ompl::base::RealVectorStateSpace>(4);

    // Set bounds for the state space
    ompl::base::RealVectorBounds bounds(4);

    // x and y bounds (match the street environment)
    bounds.setLow(0, -6);    // x min
    bounds.setHigh(0, 6);    // x max
    bounds.setLow(1, -11);   // y min
    bounds.setHigh(1, 11);   // y max

    // theta bounds (can be any angle)
    bounds.setLow(2, -M_PI);
    bounds.setHigh(2, M_PI);

    // velocity bounds (choose reasonable values)
    bounds.setLow(3, -5);    // can reverse at up to -5
    bounds.setHigh(3, 5);    // can go forward at up to 5

    space->setBounds(bounds);

    // Register projection for KPIECE
    space->registerDefaultProjection(std::make_shared<CarProjection>(space.get()));

    // Step 2: Create the control space (2D: omega, accel)
    auto cspace = std::make_shared<ompl::control::RealVectorControlSpace>(space, 2);

    // Set bounds for control (passed as parameters)
    ompl::base::RealVectorBounds cbounds(2);
    cbounds.setLow(0, -omega_bound);     // min angular velocity
    cbounds.setHigh(0, omega_bound);     // max angular velocity
    cbounds.setLow(1, -accel_bound);     // min acceleration
    cbounds.setHigh(1, accel_bound);     // max acceleration
    cspace->setBounds(cbounds);

    std::cout << "Control bounds: omega ∈ [" << -omega_bound << ", " << omega_bound
              << "], accel ∈ [" << -accel_bound << ", " << accel_bound << "]" << std::endl;

    // Step 3: Create SimpleSetup
    auto ss = std::make_shared<ompl::control::SimpleSetup>(cspace);

    // Step 4: Set up the ODE solver
    auto odeSolver = std::make_shared<ompl::control::ODEBasicSolver<>>(ss->getSpaceInformation(), &carODE);
    ss->setStatePropagator(ompl::control::ODESolver::getStatePropagator(odeSolver));

    // Step 5: Set state validity checker
    // Must check: 1) velocity within bounds, 2) no collision
    ss->setStateValidityChecker([&obstacles](const ompl::base::State *state) {
        const auto *s = state->as<ompl::base::RealVectorStateSpace::StateType>();
        double x = s->values[0];
        double y = s->values[1];
        double v = s->values[3];

        // Check velocity bounds
        if (std::abs(v) > 5.0) return false;

        // Check collision (treat car as a point)
        return isValidPoint(x, y, obstacles);
    });

    // Step 6: Set start and goal states
    // Environment: Fully enclosed street with 4 boundary walls + 2 interior obstacles
    // Free space: Must navigate around interior obstacles
    // Left obstacle: x=[-4,-1], y=[-3,1]
    // Right obstacle: x=[1,4], y=[2,5]

    // Start: Bottom left, must navigate up and around obstacles
    ompl::base::ScopedState<> start(space);
    start[0] = -3.0;         // x (left side, clear of left wall at x=-5)
    start[1] = -4.5;         // y (below left obstacle which ends at y=-3)
    start[2] = M_PI / 2.0;   // theta (facing up/north)
    start[3] = 0.0;          // v (at rest)

    // Goal: Top right, requires navigating through/around obstacles
    ompl::base::ScopedState<> goal(space);
    goal[0] = 2.5;           // x (right side, between obstacles)
    goal[1] = 4.5;           // y (top area, clear of obstacles)
    goal[2] = M_PI / 2.0;    // theta (facing up)
    goal[3] = 0.0;           // v (at rest)

    // Set with threshold (harder problem, need bigger threshold)
    ss->setStartAndGoalStates(start, goal, 1.0);

    return ss;
}

void planCar(ompl::control::SimpleSetupPtr &ss, int choice)
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
        std::ofstream fout("car_path.txt");
        // Convert to geometric path for easier plotting
        path.asGeometric().printAsMatrix(fout);
        fout.close();
        std::cout << "Path saved to car_path.txt" << std::endl;
    }
    else
    {
        std::cout << "No solution found" << std::endl;
    }
}

void benchmarkCar(ompl::control::SimpleSetupPtr &/* ss */)
{
    // TODO: Do some benchmarking for the car
}

int main(int /* argc */, char ** /* argv */)
{
    std::vector<Rectangle> obstacles;
    makeStreet(obstacles);

    int choice;
    do
    {
        std::cout << "Plan or Benchmark? " << std::endl;
        std::cout << " (1) Plan" << std::endl;
        std::cout << " (2) Benchmark" << std::endl;

        std::cin >> choice;
    } while (choice < 1 || choice > 2);

    // Ask user which control configuration to use
    int which;
    do
    {
        std::cout << "\nControl Configuration? " << std::endl;
        std::cout << " (1)  ω=0.5,  a=1   (Very Hard)" << std::endl;
        std::cout << " (2)  ω=1,    a=1.5 (Hard)" << std::endl;
        std::cout << " (3)  ω=1,    a=2   (Medium-Hard)" << std::endl;
        std::cout << " (4)  ω=1.5,  a=2   (Medium)" << std::endl;
        std::cout << " (5)  ω=2,    a=3   (Medium - Recommended)" << std::endl;
        std::cout << " (6)  ω=2.5,  a=3   (Medium-Easy)" << std::endl;
        std::cout << " (7)  ω=3,    a=4   (Easy)" << std::endl;
        std::cout << " (8)  ω=3,    a=5   (Easier)" << std::endl;
        std::cout << " (9)  ω=4,    a=5   (Very Easy)" << std::endl;
        std::cout << " (10) ω=5,    a=6   (Easiest)" << std::endl;

        std::cin >> which;
    } while (which < 1 || which > 10);

    // Define the 10 configurations [omega_bound, accel_bound]
    double configs[10][2] = {
        {0.5, 1.0},   // 1: Very Hard
        {1.0, 1.5},   // 2: Hard
        {1.0, 2.0},   // 3: Medium-Hard
        {1.5, 2.0},   // 4: Medium
        {2.0, 3.0},   // 5: Medium (Recommended)
        {2.5, 3.0},   // 6: Medium-Easy
        {3.0, 4.0},   // 7: Easy
        {3.0, 5.0},   // 8: Easier
        {4.0, 5.0},   // 9: Very Easy
        {5.0, 6.0}    // 10: Easiest
    };

    double omega_bound = configs[which - 1][0];
    double accel_bound = configs[which - 1][1];

    ompl::control::SimpleSetupPtr ss = createCar(obstacles, omega_bound, accel_bound);

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

        planCar(ss, planner);
    }
    // Benchmarking
    else if (choice == 2)
        benchmarkCar(ss);

    else
        std::cerr << "How did you get here? Invalid choice." << std::endl;

    return 0;
}
