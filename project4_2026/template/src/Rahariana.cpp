///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 4
// Authors: Rahariana - Custom Greedy RRT Variant
//////////////////////////////////////

#include "Rahariana.h"
#include <ompl/base/goals/GoalSampleableRegion.h>
#include <ompl/tools/config/SelfConfig.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <limits>
#include <algorithm>
#include <functional>

ompl::control::Rahariana::Rahariana(const SpaceInformationPtr &si) : base::Planner(si, "Rahariana")
{
    specs_.approximateSolutions = true;
    siC_ = si.get();
    sampler_ = si_->allocStateSampler();

    nn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(this));
    nn_->setDistanceFunction([this](const Motion *a, const Motion *b) {
        return si_->distance(a->state, b->state);
    });
}

ompl::control::Rahariana::~Rahariana()
{
    clear();
}

void ompl::control::Rahariana::clear()
{
    Planner::clear();

    std::vector<Motion *> motions;
    if (nn_)
        nn_->list(motions);

    for (auto *motion : motions)
        freeMotion(motion);

    if (nn_)
        nn_->clear();

    lastGoalMotion_ = nullptr;
}

void ompl::control::Rahariana::freeMotion(Motion *motion)
{
    if (motion)
    {
        if (motion->state)
            si_->freeState(motion->state);
        if (motion->control)
            siC_->freeControl(motion->control);
        delete motion;
    }
}

std::vector<ompl::control::Rahariana::Motion *> ompl::control::Rahariana::greedyExtend(Motion *from, base::State *target)
{
    std::vector<Motion *> chain;
    std::vector<Motion *> allCreated;  // Track all created motions for cleanup
    Motion *current = from;
    int previousBestControlIdx = -1;  // Track which control was best at previous step

    // Get control space bounds
    const auto *cspace = siC_->getControlSpace()->as<RealVectorControlSpace>();
    const base::RealVectorBounds &bounds = cspace->getBounds();
    unsigned int dim = cspace->getDimension();

    while (true)
    {
        double currentDist = si_->distance(current->state, target);

        Motion *bestMotion = nullptr;
        double bestDist = currentDist;
        Control *bestControl = nullptr;
        unsigned int bestSteps = 0;
        int bestControlIdx = -1;  // Track which control is best
        int validControls = 0;  // Count how many controls were collision-free

        // Try 11 evenly-spaced values for first dimension (omega for car, torque for pendulum)
        // For other dimensions (accel for car), sample randomly
        for (int i = 0; i < numControlSamples_; ++i)
        {
            Control *control = siC_->allocControl();
            auto *rcontrol = control->as<RealVectorControlSpace::ControlType>();

            // First dimension: evenly spaced
            rcontrol->values[0] = bounds.low[0] + i * (bounds.high[0] - bounds.low[0]) / (numControlSamples_ - 1);

            // Other dimensions: random
            for (unsigned int d = 1; d < dim; ++d)
                rcontrol->values[d] = rng_.uniformReal(bounds.low[d], bounds.high[d]);

            // Adaptive step size based on distance to target
            // Close to target: use small steps to avoid overshooting
            // Far from target: use large steps to move faster
            double distToTarget = si_->distance(current->state, target);
            int stepSizes[3];
            if (distToTarget < 2.0) {
                // Very close: only use small steps
                stepSizes[0] = 1;
                stepSizes[1] = 1;
                stepSizes[2] = 1;
            } else if (distToTarget < 5.0) {
                // Close: use medium and small steps
                stepSizes[0] = 5;
                stepSizes[1] = 1;
                stepSizes[2] = 1;
            } else {
                // Far: try large, medium, then small
                stepSizes[0] = 10;
                stepSizes[1] = 5;
                stepSizes[2] = 1;
            }

            for (int stepSize : stepSizes)
            {
                // Allocate state for result
                base::State *result = si_->allocState();

                // Propagate with this step size
                unsigned int steps = siC_->propagateWhileValid(current->state, control, stepSize, result);

                // Check if this gets us closer and is valid
                if (steps > 0)
                {
                    validControls++;
                    double dist = si_->distance(result, target);
                    if (dist < bestDist)
                    {
                        // This is better!
                        bestDist = dist;

                        // Free old best if exists
                        if (bestMotion)
                        {
                            si_->freeState(bestMotion->state);
                            siC_->freeControl(bestMotion->control);
                            delete bestMotion;
                        }
                        if (bestControl)
                            siC_->freeControl(bestControl);

                        // Save this as new best
                        bestMotion = new Motion(siC_);
                        si_->copyState(bestMotion->state, result);
                        bestControl = siC_->allocControl();
                        siC_->copyControl(bestControl, control);
                        bestSteps = steps;
                        bestControlIdx = i;

                        si_->freeState(result);
                        break;  // Found improvement with this control, no need to try smaller steps
                    }
                    else
                    {
                        // Not better, free it and try smaller step size
                        si_->freeState(result);
                    }
                }
                else
                {
                    // Invalid propagation, free it and try smaller step size
                    si_->freeState(result);
                }
            }

            siC_->freeControl(control);
        }

        // Check if we found a better state
        if (bestMotion && bestDist < currentDist)
        {
            // Set up the motion
            bestMotion->control = bestControl;
            bestMotion->steps = bestSteps;
            bestMotion->parent = current;

            // Track all created motions
            allCreated.push_back(bestMotion);

            // Only add to chain if control direction CHANGED (this is a turning point)
            if (bestControlIdx != previousBestControlIdx)
            {
                chain.push_back(bestMotion);

                // Check chain length limit
                if (chain.size() >= (size_t)maxChainLength_)
                {
                    // Prune: keep every other node
                    std::vector<Motion *> pruned;
                    for (size_t j = 0; j < chain.size(); j += 2)
                        pruned.push_back(chain[j]);

                    // Free the ones we're not keeping
                    for (size_t j = 1; j < chain.size(); j += 2)
                    {
                        // Remove from allCreated too
                        auto it = std::find(allCreated.begin(), allCreated.end(), chain[j]);
                        if (it != allCreated.end())
                            allCreated.erase(it);
                        freeMotion(chain[j]);
                    }

                    chain = pruned;
                }
            }

            // Continue extending from new position regardless
            current = bestMotion;
            previousBestControlIdx = bestControlIdx;
        }
        else
        {
            // No improvement, stop
            static int emptyChainDebugCount = 0;
            if (emptyChainDebugCount < 5)
            {
                OMPL_INFORM("greedyExtend stopping: tried %d controls (%d valid), bestDist=%.3f >= currentDist=%.3f, chain has %d nodes",
                           numControlSamples_, validControls, bestDist, currentDist, (int)chain.size());
                emptyChainDebugCount++;
            }

            if (bestMotion)
            {
                freeMotion(bestMotion);
                if (bestControl)
                    siC_->freeControl(bestControl);
            }
            break;
        }
    }

    // Free any motions that were created but not added to chain (intermediate steps with same control)
    for (auto *motion : allCreated)
    {
        // Check if this motion is in the chain
        bool inChain = std::find(chain.begin(), chain.end(), motion) != chain.end();
        if (!inChain)
        {
            freeMotion(motion);
        }
    }

    return chain;
}

ompl::base::PlannerStatus ompl::control::Rahariana::solve(const base::PlannerTerminationCondition &ptc)
{
    checkValidity();

    base::Goal *goal = pdef_->getGoal().get();

    // Add start states
    while (const base::State *st = pis_.nextStart())
    {
        auto *motion = new Motion(siC_);
        si_->copyState(motion->state, st);
        siC_->nullControl(motion->control);
        nn_->add(motion);
    }

    if (nn_->size() == 0)
    {
        OMPL_ERROR("%s: No valid initial states!", getName().c_str());
        return base::PlannerStatus::INVALID_START;
    }

    auto *rmotion = new Motion(siC_);

    int iteration = 0;
    int totalNodesAdded = 0;

    while (!ptc)
    {
        // Sample random state
        sampler_->sampleUniform(rmotion->state);

        // Find nearest
        Motion *nmotion = nn_->nearest(rmotion);

        // Greedily extend toward random state
        std::vector<Motion *> chain = greedyExtend(nmotion, rmotion->state);

        // Add all motions in chain to tree (these are the turning points)
        for (auto *motion : chain)
        {
            nn_->add(motion);
            totalNodesAdded++;

            // Check if reached goal
            if (goal->isSatisfied(motion->state))
            {
                lastGoalMotion_ = motion;
                break;
            }
        }

        iteration++;
        if (iteration % 100 == 0)
        {
            OMPL_INFORM("Iteration %d: %d nodes added (%d in chain), %d total nodes in tree",
                       iteration, totalNodesAdded, (int)chain.size(), (int)nn_->size());
        }

        // Debug: print first few empty chains to understand why
        if (chain.empty() && iteration <= 500)
        {
            OMPL_DEBUG("Iteration %d: Empty chain from nearest node. Investigating...", iteration);
        }

        if (lastGoalMotion_)
            break;
    }

    // Cleanup
    si_->freeState(rmotion->state);
    siC_->freeControl(rmotion->control);
    delete rmotion;

    bool solved = false;
    if (lastGoalMotion_)
    {
        solved = true;

        // Construct path
        std::vector<Motion *> mpath;
        Motion *solution = lastGoalMotion_;
        while (solution)
        {
            mpath.push_back(solution);
            solution = solution->parent;
        }

        auto path(std::make_shared<PathControl>(si_));
        for (int i = mpath.size() - 1; i >= 0; --i)
        {
            if (mpath[i]->parent)
                path->append(mpath[i]->state, mpath[i]->control,
                           mpath[i]->steps * siC_->getPropagationStepSize());
            else
                path->append(mpath[i]->state);
        }

        pdef_->addSolutionPath(path, false, 0.0, getName());
    }

    return solved ? base::PlannerStatus::EXACT_SOLUTION : base::PlannerStatus::TIMEOUT;
}
