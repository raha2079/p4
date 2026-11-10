///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 4
// Authors: Rahariana - Custom Greedy RRT Variant
//////////////////////////////////////

#ifndef RAHARIANA_H
#define RAHARIANA_H

#include <ompl/control/planners/PlannerIncludes.h>
#include <ompl/datastructures/NearestNeighbors.h>
#include <vector>

namespace ompl
{
    namespace control
    {
        /**
         * @brief Rahariana Planner - Greedy Control Selection Variant
         *
         * Custom planner that greedily selects controls to get as close as possible
         * to random samples. Unlike RG-RRT which propagates once, Rahariana keeps
         * propagating using the best control until no further progress is made.
         */
        class Rahariana : public base::Planner
        {
        public:
            Rahariana(const SpaceInformationPtr &si);
            ~Rahariana() override;

            base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc) override;
            void clear() override;

            /** Set number of controls to try at each step */
            void setNumControlSamples(int num)
            {
                numControlSamples_ = num;
            }

            int getNumControlSamples() const
            {
                return numControlSamples_;
            }

            /** Set max chain length before pruning */
            void setMaxChainLength(int length)
            {
                maxChainLength_ = length;
            }

            int getMaxChainLength() const
            {
                return maxChainLength_;
            }

        protected:
            class Motion
            {
            public:
                Motion() = default;
                Motion(const SpaceInformation *si)
                    : state(si->allocState()), control(si->allocControl())
                {
                }

                base::State *state{nullptr};
                Control *control{nullptr};
                unsigned int steps{0};
                Motion *parent{nullptr};
            };

            /** Greedily extend from a motion toward target using best controls */
            std::vector<Motion *> greedyExtend(Motion *from, base::State *target);

            /** Free a motion */
            void freeMotion(Motion *motion);

            base::StateSamplerPtr sampler_;
            const SpaceInformation *siC_;
            std::shared_ptr<NearestNeighbors<Motion *>> nn_;
            Motion *lastGoalMotion_{nullptr};

            int numControlSamples_{11};
            int maxChainLength_{50};  // Prune chains longer than this
            RNG rng_;
        };

    }  // namespace control
}  // namespace ompl

#endif
