///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 4
// Authors: Raha Gharehdahgi(rg152), Ariana Diaz(ad316)
//////////////////////////////////////

#ifndef RGRRT_H
#define RGRRT_H

#include <ompl/control/planners/PlannerIncludes.h>
#include <ompl/datastructures/NearestNeighbors.h>
#include <vector>

namespace ompl
{
    namespace control
    {
        /**
         * @brief Reachability-Guided RRT (RG-RRT)
         *
         * RG-RRT extends RRT by maintaining a reachable set for each state in the tree.
         * Random samples are only accepted if they are in a "reachable direction" from
         * the nearest neighbor, making the planner more efficient for systems with
         * dynamical constraints.
         */
        class RGRRT : public base::Planner
        {
        public:
            /** Constructor */
            RGRRT(const SpaceInformationPtr &si);

            /** Destructor */
            ~RGRRT() override;

            /** Solve the planning problem */
            base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc) override;

            /** Clear all internal data structures */
            void clear() override;

            /** Set the duration used for computing reachable sets */
            void setPropagationStepSize(double stepSize)
            {
                propagationStepSize_ = stepSize;
            }

            /** Get the propagation step size */
            double getPropagationStepSize() const
            {
                return propagationStepSize_;
            }

            /** Set the number of controls to sample for reachable set */
            void setNumControlSamples(int num)
            {
                numControlSamples_ = num;
            }

            /** Get the number of control samples */
            int getNumControlSamples() const
            {
                return numControlSamples_;
            }

        protected:
            /** Representation of a motion (tree node) */
            class Motion
            {
            public:
                Motion() = default;

                /** Constructor that allocates state and control */
                Motion(const SpaceInformation *si)
                    : state(si->allocState()), control(si->allocControl())
                {
                }

                ~Motion() = default;

                /** The state contained by this motion */
                base::State *state{nullptr};

                /** The control that was applied to get to this state from the parent */
                Control *control{nullptr};

                /** The number of steps the control was applied */
                unsigned int steps{0};

                /** The parent motion in the tree */
                Motion *parent{nullptr};

                /** The reachable set from this state (NEW for RG-RRT) */
                std::vector<base::State *> reachableSet;
            };

            /** Compute the reachable set for a given motion */
            void computeReachableSet(Motion *motion);

            /** Check if qrand is reachable from qnear
             * Returns true if any state in R(qnear) is closer to qrand than qnear itself
             */
            bool isReachable(Motion *qnear, base::State *qrand) const;

            /** Free the memory allocated by a motion (state, control, and reachable set) */
            void freeMotion(Motion *motion);

            /** State sampler */
            base::StateSamplerPtr sampler_;

            /** Control sampler */
            DirectedControlSamplerPtr controlSampler_;

            /** The control space information */
            const SpaceInformation *siC_;

            /** A nearest-neighbors data structure containing the tree of motions */
            std::shared_ptr<NearestNeighbors<Motion *>> nn_;

            /** The most recent goal motion. Used for efficient retrieval of solution. */
            Motion *lastGoalMotion_{nullptr};

            /** The duration to use for propagating controls when computing reachable sets */
            double propagationStepSize_{0.1};

            /** Number of controls to sample for reachable set (default 11 as per PDF) */
            int numControlSamples_{11};

            /** Random number generator */
            RNG rng_;
        };

    }  // namespace control
}  // namespace ompl

#endif
