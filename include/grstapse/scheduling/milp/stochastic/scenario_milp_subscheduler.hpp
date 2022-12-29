/*
 * Graphically Recursive Simultaneous Task Allocation, Planning,
 * Scheduling, and Execution
 *
 * Copyright (C) 2020-2022
 *
 * Author: Andrew Messing
 * Author: Glen Neville
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */
#pragma once

// Local
#include "grstapse/scheduling/milp/deterministic/deterministic_milp_scheduler_base.hpp"

namespace grstapse
{
    /**!
     * Subscheduler for a specific scenario used in the StochasticMilpScheduler
     *
     * \see StochasticMilpScheduler
     */
    class ScenarioMilpSubscheduler : public DeterministicMilpSchedulerBase
    {
       public:
        //! Constructor
        ScenarioMilpSubscheduler(
            unsigned int index,
            const std::shared_ptr<const SchedulerProblemInputs>& problem_inputs,
            robin_hood::unordered_map<std::string, MutexConstraintInfo>& reduced_mutex_constraints);

       protected:
        //! \copydoc DeterministicMilpSchedulerBase
        [[nodiscard]] std::string createTaskStartName(unsigned int task_nr) const override;

        //! \copydoc DeterministicMilpSchedulerBase
        [[nodiscard]] std::string createTaskFinishName(unsigned int task_nr) const override;

        //! \copydoc DeterministicMilpSchedulerBase
        [[nodiscard]] std::string createDurationConstraintName(unsigned int task_nr) const override;

        //! \copydoc DeterministicMilpSchedulerBase
        [[nodiscard]] float computeTaskDuration(
            unsigned int task_nr,
            const std::vector<std::shared_ptr<const Robot>>& coalition) final override;

        //! \copydoc DeterministicMilpSchedulerBase
        [[nodiscard]] std::string createPrecedenceConstraintName(unsigned int i, unsigned int j) const override;

        //! \copydoc DeterministicMilpSchedulerBase
        void addMutexConstraint(GRBModel& model,
                                const unsigned int i,
                                const double i_to_j_transition_duration,
                                const unsigned int j,
                                const double j_to_i_transition_duration) final override;

        //! \copydoc DeterministicMilpSchedulerBase
        [[nodiscard]] std::string createInitialTransitionConstraintName(unsigned int task_nr) const override;

        //! \copydoc DeterministicMilpSchedulerBase
        [[nodiscard]] bool isInitialTransitionMemoized(const std::shared_ptr<const ConfigurationBase>& configuration,
                                                       const std::shared_ptr<const Robot>& robot) const final override;

        //! \copydoc DeterministicMilpSchedulerBase
        [[nodiscard]] float computeInitialTransitionDuration(
            const std::shared_ptr<const ConfigurationBase>& configuration,
            const std::shared_ptr<const Robot>& robot) const final override;

        //! \copydoc DeterministicMilpSchedulerBase
        [[nodiscard]] float computeInitialTransitionHeuristicDuration(
            const std::shared_ptr<const ConfigurationBase>& configuration,
            const std::shared_ptr<const Robot>& robot) const override;

        //! \copydoc DeterministicMilpSchedulerBase
        [[nodiscard]] bool isTransitionMemoized(const std::shared_ptr<const ConfigurationBase>& initial_configuration,
                                                const std::shared_ptr<const ConfigurationBase>& goal_configuration,
                                                const std::shared_ptr<const Robot>& robot) const override;

        //! \copydoc DeterministicMilpSchedulerBase
        [[nodiscard]] float computeTransitionDuration(
            const std::shared_ptr<const ConfigurationBase>& initial_configuration,
            const std::shared_ptr<const ConfigurationBase>& goal_configuration,
            const std::shared_ptr<const Robot>& robot) const override;

        //! \copydoc DeterministicMilpSchedulerBase
        [[nodiscard]] float computeTransitionHeuristicDuration(
            const std::shared_ptr<const ConfigurationBase>& initial_configuration,
            const std::shared_ptr<const ConfigurationBase>& goal_configuration,
            const std::shared_ptr<const Robot>& robot) const override;

        //! \copydoc MilpSchedulerBase
        bool createObjective(GRBModel& model) override;

       protected:
        unsigned int m_index;

       private:
        std::shared_ptr<const ScheduleBase> computeSchedule() override;

        friend class StochasticMilpScheduler;
    };

}  // namespace grstapse