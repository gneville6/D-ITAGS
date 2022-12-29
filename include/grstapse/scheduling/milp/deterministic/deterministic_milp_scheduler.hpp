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

// Global
#include <list>
#include <map>
#include <memory>
#include <set>
#include <tuple>
// External
#include <gurobi_c++.h>
#include <robin_hood/robin_hood.hpp>
// Local
#include "grstapse/scheduling/milp/deterministic/deterministic_milp_scheduler_base.hpp"
#include "grstapse/scheduling/milp/mutex_constraint_info.hpp"
#include "grstapse/scheduling/milp/robot_task_transition_info.hpp"
#include "grstapse/scheduling/milp/task_variable_info.hpp"

namespace grstapse {
    // Forward Declaration
    class DeterministicSchedule;

    /**!
     * \brief Uses a MILP formulation to solve a deterministic robot scheduling problem
     *
     * \note We use gurobi as our MILP solver
     */
    class DeterministicMilpScheduler : public DeterministicMilpSchedulerBase {
    public:
        /**!
         * \brief Constructor
         *
         * \param problem_inputs Inputs for a scheduling problem
         */
        explicit DeterministicMilpScheduler(const std::shared_ptr<const SchedulerProblemInputs> &problem_inputs);

        void recomputEnviroment();

    protected:
        //! \copydoc SchedulerBase
        std::shared_ptr<const ScheduleBase> computeSchedule() override;

        //! \copydoc DeterministicMilpSchedulerBase
        [[nodiscard]] std::string createTaskStartName(unsigned int task_nr) const final override;

        //! \copydoc DeterministicMilpSchedulerBase
        [[nodiscard]] std::string createTaskFinishName(unsigned int task_nr) const final override;

        //! \copydoc DeterministicMilpSchedulerBase
        [[nodiscard]] std::string createDurationConstraintName(unsigned int task_nr) const final override;

        //! \copydoc DeterministicMilpSchedulerBase
        [[nodiscard]] std::string createPrecedenceConstraintName(unsigned int i, unsigned int j) const final override;

        //! \copydoc DeterministicMilpSchedulerBase
        void addMutexConstraint(GRBModel &model,
                                unsigned int i,
                                double i_to_j_transition_duration,
                                unsigned int j,
                                double j_to_i_transition_duration) final override;

        //! \copydoc DeterministicMilpSchedulerBase
        [[nodiscard]] float computeTaskDuration(unsigned int task_nr,
                                                const std::vector<std::shared_ptr<const Robot>> &coalition) override;

        // region computeInitialTransitionHeuristicDurations
        //! \copydoc DeterministicMilpSchedulerBase
        [[nodiscard]] std::string createInitialTransitionConstraintName(unsigned int task_nr) const final override;

        //! \copydoc DeterministicMilpSchedulerBase
        [[nodiscard]] bool isInitialTransitionMemoized(const std::shared_ptr<const ConfigurationBase> &configuration,
                                                       const std::shared_ptr<const Robot> &robot) const override;

        //! \copydoc DeterministicMilpSchedulerBase
        [[nodiscard]] float computeInitialTransitionDuration(
                const std::shared_ptr<const ConfigurationBase> &configuration,
                const std::shared_ptr<const Robot> &robot) const override;
        // endregion

        // region computeTransitionHeuristicDurations
        //! \copydoc DeterministicMilpSchedulerBase
        [[nodiscard]] bool isTransitionMemoized(const std::shared_ptr<const ConfigurationBase> &initial_configuration,
                                                const std::shared_ptr<const ConfigurationBase> &goal_configuration,
                                                const std::shared_ptr<const Robot> &robot) const override;

        //! \copydoc DeterministicMilpSchedulerBase
        [[nodiscard]] float computeTransitionDuration(
                const std::shared_ptr<const ConfigurationBase> &initial_configuration,
                const std::shared_ptr<const ConfigurationBase> &goal_configuration,
                const std::shared_ptr<const Robot> &robot) const override;
        // endregion

        //! \copydoc MilpSchedulerBase
        bool createObjective(GRBModel &model) final override;

        /**!
         * Checks if a solution has been found (if there are no heuristic values in the MILP equations)
         *
         * \returns true if there are no heuristic values in the MILP equations, false otherwise
         */
        bool checkAndUpdateTransitions();

        //! Creates a schedule from the solved variables
        virtual std::shared_ptr<const DeterministicSchedule> createSchedule(GRBModel &model);

        //! Note: Needed for DeterministicMilpScheduler::m_reduced_mutex_constraints's reference
        robin_hood::unordered_map<std::string, MutexConstraintInfo> m_placeholder_reduced_mutex_constraints;
        GRBVar m_makespan;
    };
}  // namespace grstapse