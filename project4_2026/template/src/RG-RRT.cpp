///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 4
// Authors: Raha Gharehdahgi(rg152), Ariana Diaz(ad316)
//////////////////////////////////////

#include "RG-RRT.h"
#include <ompl/base/goals/GoalSampleableRegion.h>
#include <ompl/tools/config/SelfConfig.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <limits>

ompl::control::RGRRT::RGRRT(const SpaceInformationPtr &si) : base::Planner(si, "RGRRT")
{
    specs_.approximateSolutions = true;

    // Get the control space information
    siC_ = si.get();

    // Create the state sampler
    sampler_ = si_->allocStateSampler();

    // Create the control sampler (directed control sampler for steering toward target)
    controlSampler_ = siC_->allocDirectedControlSampler();

    // Set up the nearest neighbors data structure
    nn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(this));
    nn_->setDistanceFunction([this](const Motion *a, const Motion *b) {
        return si_->distance(a->state, b->state);
    });
}

ompl::control::RGRRT::~RGRRT()
{
    clear();
}

void ompl::control::RGRRT::clear()
{
    Planner::clear();

    // Free all motions
    std::vector<Motion *> motions;
    if (nn_)
        nn_->list(motions);

    for (auto *motion : motions)
        freeMotion(motion);

    if (nn_)
        nn_->clear();

    lastGoalMotion_ = nullptr;
}

void ompl::control::RGRRT::freeMotion(Motion *motion)
{
    if (motion)
    {
        // Free the state
        if (motion->state)
            si_->freeState(motion->state);

        // Free the control
        if (motion->control)
            siC_->freeControl(motion->control);

        // Free all states in the reachable set
        for (auto *state : motion->reachableSet)
            si_->freeState(state);

        // Delete the motion object
        delete motion;
    }
}

void ompl::control::RGRRT::computeReachableSet(Motion *motion)
{
    // Clear any existing reachable set
    for (auto *state : motion->reachableSet)
        si_->freeState(state);
    motion->reachableSet.clear();

    // Get control bounds
    const auto *cspace = siC_->getControlSpace()->as<RealVectorControlSpace>();
    const base::RealVectorBounds &bounds = cspace->getBounds();

    // Get the dimension of the control space (1 for pendulum, 2 for car)
    unsigned int dim = cspace->getDimension();

    // For the reachable set, we only vary the first control dimension
    // (torque for pendulum, angular velocity for car)
    double minControl = bounds.low[0];
    double maxControl = bounds.high[0];

    // Use a fixed number of steps for reachable set propagation
    // Using more steps gives a larger reachable set
    // Higher values = larger reachable set = less restrictive filtering
    // TUNING: This parameter significantly affects performance
    //   - Too small (1-3): reachability check becomes too restrictive, may timeout
    //   - Too large (20+): reachable set computation becomes expensive
    //   - Sweet spot seems to be around 5-15 depending on problem difficulty
    unsigned int steps = 10;  // Works well for most problems

    // Compute evenly spaced control values
    for (int i = 0; i < numControlSamples_; ++i)
    {
        // Allocate a control
        Control *control = siC_->allocControl();
        auto *rcontrol = control->as<RealVectorControlSpace::ControlType>();

        // Set the first dimension to evenly spaced value
        rcontrol->values[0] = minControl + i * (maxControl - minControl) / (numControlSamples_ - 1);

        // Set other dimensions to 0 (for car, we ignore acceleration in reachable set)
        for (unsigned int d = 1; d < dim; ++d)
            rcontrol->values[d] = 0.0;

        // Allocate state for result
        base::State *result = si_->allocState();

        // Propagate using propagateWhileValid
        unsigned int actualSteps = siC_->propagateWhileValid(motion->state, control, steps, result);

        // If propagation succeeded, add to reachable set
        if (actualSteps > 0)
        {
            motion->reachableSet.push_back(result);
        }
        else
        {
            // Propagation failed, free the state
            si_->freeState(result);
        }

        // Free the control
        siC_->freeControl(control);
    }
}

bool ompl::control::RGRRT::isReachable(Motion *qnear, base::State *qrand) const
{
    // If reachable set is empty, accept the sample
    // (This shouldn't happen but handle gracefully)
    if (qnear->reachableSet.empty())
        return true;

    // Compute distance from qnear to qrand
    double distNearToRand = si_->distance(qnear->state, qrand);

    // Check if any state in the reachable set is closer to qrand than qnear
    for (auto *reachableState : qnear->reachableSet)
    {
        double distReachableToRand = si_->distance(reachableState, qrand);
        if (distReachableToRand < distNearToRand)
        {
            // Found a reachable state closer to qrand
            return true;
        }
    }

    // No reachable state is closer than qnear itself
    return false;
}

ompl::base::PlannerStatus ompl::control::RGRRT::solve(const base::PlannerTerminationCondition &ptc)
{
    // Basic input checking
    checkValidity();

    // Get the goal
    base::Goal *goal = pdef_->getGoal().get();
    auto *goal_s = dynamic_cast<base::GoalSampleableRegion *>(goal);

    // Add the start state(s) to the tree
    while (const base::State *st = pis_.nextStart())
    {
        auto *motion = new Motion(siC_);
        si_->copyState(motion->state, st);
        siC_->nullControl(motion->control);

        // Compute reachable set for start state
        computeReachableSet(motion);

        // Add to tree
        nn_->add(motion);
    }

    // Check if we have a start state
    if (nn_->size() == 0)
    {
        OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
        return base::PlannerStatus::INVALID_START;
    }

    // Create a random motion for sampling
    auto *rmotion = new Motion(siC_);

    // Allocate a control for sampling
    Control *rctrl = siC_->allocControl();

    // Main planning loop
    while (!ptc)
    {
        // Sample a random state
        sampler_->sampleUniform(rmotion->state);

        // Find the nearest motion in the tree to the random motion
        Motion *nmotion = nn_->nearest(rmotion);

        // RG-RRT reachability check
        if (!isReachable(nmotion, rmotion->state))
        {
            // rmotion->state is not in a reachable direction, reject it
            continue;
        }

        // rmotion passed reachability check, now do normal RRT propagation

        // Sample a control that steers toward rmotion->state
        unsigned int cd = controlSampler_->sampleTo(rctrl, nmotion->control,
                                                     nmotion->state, rmotion->state);

        if (cd >= siC_->getMinControlDuration())
        {
            // Create a new motion
            auto *motion = new Motion(siC_);

            // Propagate and check if we reach a valid state
            cd = siC_->propagateWhileValid(nmotion->state, rctrl, cd, motion->state);

            if (cd >= siC_->getMinControlDuration())
            {
                // Copy the control that was sampled
                siC_->copyControl(motion->control, rctrl);
                motion->steps = cd;
                motion->parent = nmotion;

                // Compute reachable set for the new motion
                computeReachableSet(motion);

                // Add to tree
                nn_->add(motion);

                // Check if we reached the goal
                if (goal->isSatisfied(motion->state))
                {
                    lastGoalMotion_ = motion;
                    break;
                }
            }
            else
            {
                // Propagation failed, free the motion
                freeMotion(motion);
            }
        }
    }

    // Free temporary motion and control
    si_->freeState(rmotion->state);
    siC_->freeControl(rmotion->control);
    delete rmotion;
    siC_->freeControl(rctrl);

    // Check if we found a solution
    bool solved = false;
    bool approximate = false;

    if (lastGoalMotion_ != nullptr)
    {
        // We found a goal state
        solved = true;

        // Construct the solution path
        std::vector<Motion *> mpath;
        Motion *solution = lastGoalMotion_;

        while (solution != nullptr)
        {
            mpath.push_back(solution);
            solution = solution->parent;
        }

        // Create the path
        auto path(std::make_shared<PathControl>(si_));
        for (int i = mpath.size() - 1; i >= 0; --i)
        {
            if (mpath[i]->parent)
                path->append(mpath[i]->state, mpath[i]->control,
                           mpath[i]->steps * siC_->getPropagationStepSize());
            else
                path->append(mpath[i]->state);
        }

        pdef_->addSolutionPath(path, approximate, 0.0, getName());
    }

    return solved ? base::PlannerStatus::EXACT_SOLUTION : base::PlannerStatus::TIMEOUT;
}
