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
// External
#include <robin_hood/robin_hood.hpp>
// Local
#include "grstapse/common/utilities/custom_hashings.hpp"
#include "grstapse/scheduling/milp/milp_scheduler_base.hpp"
#include "grstapse/scheduling/milp/mutex_constraint_info.hpp"
#include "grstapse/scheduling/milp/robot_task_transition_info.hpp"
#include "grstapse/scheduling/milp/task_variable_info.hpp"

namespace grstapse
{
    // Forward Declarations
    class ConfigurationBase;
    class Robot;

    /**!
     * Abstract base class for MILP formulations to solve deterministic robot scheduling problems
     *
     * \see DeterministicMilpScheduler
     * \see QuickDeterministicMilpScheduler
     * \see ScenarioDeterministicMilpScheduler
     */
    class DeterministicMilpSchedulerBase : public MilpSchedulerBase
    {
       public:
        //! Constructor
        explicit DeterministicMilpSchedulerBase(
            const std::shared_ptr<const SchedulerProblemInputs>& problem_inputs,
            robin_hood::unordered_map<std::string, MutexConstraintInfo>& reduced_mutex_constraints);

       protected:
        /**!
         * Initializes the members of DeterministicMilpSchedulerBase
         */
        void init();

        // region createTaskDurations
        //! \copydoc MilpSchedulerBase
        bool createTaskDurations(GRBModel& model) override;

        //! \returns A name for a task start MILP variable
        [[nodiscard]] virtual std::string createTaskStartName(unsigned int task_nr) const = 0;

        //! \returns A name for a task finish MILP variable
        [[nodiscard]] virtual std::string createTaskFinishName(unsigned int task_nr) const = 0;

        //! \returns A name for a task duration MILP constraint
        [[nodiscard]] virtual std::string createDurationConstraintName(unsigned int task_nr) const = 0;

        /**!
         * Adds temporal constraints to \p model for the durations of the tasks to schedule
         *
         * This function is used for the first iteration of scheduling and has extra computations for determining
         * coalitions and durations
         *
         * \param model Reference to the model that is being constructed
         * \return Whether the constraints were successfully added
         */
        virtual bool createTaskDurationsFirstIteration(GRBModel& model);

        /**!
         * Adds temporal constraints to \p model for the durations of the tasks to schedule
         *
         * This function is used for all but the first iteration and simply resets the GRBVar's with new ones from the
         * new \p model
         *
         * \param model Reference to the model that is being constructed
         * \return Whether the constraints were successfully added
         */
        virtual bool createTaskDurationsOtherIterations(GRBModel& model);

        /**!
         * Compute the duration for a specific \p coalition performing a specific task \p task_nr
         *
         * \param task_nr The identifier for the task
         * \param coalition A list of robots that are supposed to execute the task
         *
         * \returns The duration for a specific \p coalition performing a specific task \p task_nr
         */
        virtual float computeTaskDuration(unsigned int task_nr,
                                          const std::vector<std::shared_ptr<const Robot>>& coalition) = 0;
        // endregion

        //! \copydoc MilpSchedulerBase
        bool createPrecedenceConstraints(GRBModel& model) override;

        //! \returns A name for a task precedence MILP constraint
        [[nodiscard]] virtual std::string createPrecedenceConstraintName(unsigned int i, unsigned int j) const = 0;

        // region createMutexConstraints
        //! \copydoc MilpSchedulerBase
        bool createMutexConstraints(GRBModel& model) override;

        /**!
         * Adds mutex constraints to \p model
         *
         * This function is used for the first iteration and reduces the set of mutex constraints by eliminating those
         * that would be cut through precedence constraints
         *
         * \param model Reference to the model that is being constructed
         * \returns Whether the constraints were successfully added
         */
        virtual bool createMutexConstraintsFirstIteration(GRBModel& model);

        /**!
         * Adds mutex constraints to \p model
         *
         * This function is used for all but the first iteration and simply creates the constraints for the new model
         *
         * \param model Reference to the model that is being constructed
         * \returns Whether the constraints were successfully added
         */
        virtual bool createMutexConstraintsOtherIterations(GRBModel& model);

        /**!
         * Adds a single mutex constraint to the \p model
         *
         * \param i Index for a task
         * \param i_to_j_transition_duration Time needed to transition from task i to task j
         * \param j Index for another task
         * \param j_to_i_transition_duration Time needed to transition from task j to task i
         */
        virtual void addMutexConstraint(GRBModel& model,
                                        const unsigned int i,
                                        const double i_to_j_transition_duration,
                                        const unsigned int j,
                                        const double j_to_i_transition_duration) = 0;
        // endregion

        //! \copydoc MilpSchedulerBase
        bool createInitialTransitions(GRBModel& model) override;

        //! \returns A name for a initial transition MILP constraint
        [[nodiscard]] virtual std::string createInitialTransitionConstraintName(unsigned int task_nr) const = 0;

        // region computeInitialTransitionHeuristicDurations
        /**!
         * Computes the initial transition durations based on euclidean distance
         */
        bool computeInitialTransitionHeuristicDurations();

        /**!
         *
         * \param configuration
         * \param robot
         *
         * \returns
         */
        [[nodiscard]] virtual bool isInitialTransitionMemoized(
            const std::shared_ptr<const ConfigurationBase>& configuration,
            const std::shared_ptr<const Robot>& robot) const = 0;

        /**!
         *
         * \param configuration
         * \param robot
         *
         * \returns
         */
        [[nodiscard]] virtual float computeInitialTransitionDuration(
            const std::shared_ptr<const ConfigurationBase>& configuration,
            const std::shared_ptr<const Robot>& robot) const = 0;

        /**!
         *
         * \param configuration
         * \param robot
         *
         * \returns
         */
        [[nodiscard]] virtual float computeInitialTransitionHeuristicDuration(
            const std::shared_ptr<const ConfigurationBase>& configuration,
            const std::shared_ptr<const Robot>& robot) const;
        // endregion

        // region computeTransitionHeuristicDuration
        /**!
         * Computes the transition durations based on euclidean distance
         */
        virtual bool computeTransitionHeuristicDurations();

        /**!
         *
         * \param initial_configuration
         * \param goal_configuration
         * \param robot
         *
         * \returns
         */
        [[nodiscard]] virtual bool isTransitionMemoized(
            const std::shared_ptr<const ConfigurationBase>& initial_configuration,
            const std::shared_ptr<const ConfigurationBase>& goal_configuration,
            const std::shared_ptr<const Robot>& robot) const = 0;

        /**!
         *
         * \param initial_configuration
         * \param goal_configuration
         * \param robot
         *
         * \returns
         */
        [[nodiscard]] virtual float computeTransitionDuration(
            const std::shared_ptr<const ConfigurationBase>& initial_configuration,
            const std::shared_ptr<const ConfigurationBase>& goal_configuration,
            const std::shared_ptr<const Robot>& robot) const = 0;

        /**!
         *
         * \param initial_configuration
         * \param goal_configuration
         * \param robot
         *
         * \returns
         */
        [[nodiscard]] virtual float computeTransitionHeuristicDuration(
            const std::shared_ptr<const ConfigurationBase>& initial_configuration,
            const std::shared_ptr<const ConfigurationBase>& goal_configuration,
            const std::shared_ptr<const Robot>& robot) const;
        // endregion

        /**!
         * Checks if a transition exists between two tasks
         *
         * \param i Index for a task
         * \param j Index for another task
         *
         * \returns A pair [whether a transition was found, the duration of the transition]
         */
        [[nodiscard]] virtual std::pair<bool, float> checkTransitionFeasibility(unsigned int i, unsigned int j);

        std::vector<float> m_task_durations;
        std::vector<TaskVariableInfo> m_tasks_timepoints;
        std::vector<TaskTransitionInfo> m_initial_transition_info;
        std::vector<std::vector<TaskTransitionInfo>> m_transition_info;
        std::unique_ptr<GRBVar> m_task_finishes;  //!< For makespan
        robin_hood::unordered_set<std::pair<unsigned int, unsigned int>> m_mp_induced_precedence_constraints;
        robin_hood::unordered_map<std::string, MutexConstraintInfo>& m_reduced_mutex_constraints;
    };

}  // namespace grstapse