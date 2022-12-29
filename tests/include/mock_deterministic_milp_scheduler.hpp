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

// Project
#include <grstapse/scheduling/milp/deterministic/deterministic_milp_scheduler.hpp>
#include <grstapse/scheduling/scheduler_problem_inputs.hpp>

namespace grstapse::mocks
{
    /**!
     * Mock to test functions from MilpScheduler
     */
    class MockDeterministicMilpScheduler : public DeterministicMilpScheduler
    {
       public:
        //! Constructor
        explicit MockDeterministicMilpScheduler(const std::shared_ptr<const SchedulerProblemInputs>& problem_inputs)
            : DeterministicMilpScheduler(problem_inputs)
        {}

        //! A list of durations of each task
        [[nodiscard]] inline const std::vector<float>& taskDurations() const
        {
            return m_task_durations;
        }

        [[nodiscard]] inline const std::vector<TaskVariableInfo>& taskTimepoints() const
        {
            return m_tasks_timepoints;
        }

        [[nodiscard]] inline const robin_hood::unordered_set<std::pair<unsigned int, unsigned int>>&
        mpInducedPrecedenceConstraints() const
        {
            return m_mp_induced_precedence_constraints;
        }

        [[nodiscard]] inline const std::vector<TaskTransitionInfo>& initialTransitionInfo() const
        {
            return m_initial_transition_info;
        }

        [[nodiscard]] inline const std::vector<std::vector<TaskTransitionInfo>>& transitionInfo() const
        {
            return m_transition_info;
        }

        [[nodiscard]] inline const Eigen::MatrixXf& allocation() const
        {
            return m_problem_inputs->allocation();
        }

        [[nodiscard]] inline std::vector<std::shared_ptr<const Task>> tasks() const
        {
            return m_problem_inputs->planTasks();
        }

        [[nodiscard]] inline const robin_hood::unordered_map<std::string, MutexConstraintInfo>&
        reducedMutexConstraints() const
        {
            return m_reduced_mutex_constraints;
        }

        using DeterministicMilpScheduler::checkAndUpdateTransitions;
        using DeterministicMilpScheduler::computeInitialTransitionHeuristicDurations;
        using DeterministicMilpScheduler::computeTransitionHeuristicDurations;
        using DeterministicMilpScheduler::createInitialTransitions;
        using DeterministicMilpScheduler::createModel;
        using DeterministicMilpScheduler::createMutexConstraints;
        using DeterministicMilpScheduler::createMutexConstraintsFirstIteration;
        using DeterministicMilpScheduler::createMutexConstraintsOtherIterations;
        using DeterministicMilpScheduler::createObjective;
        using DeterministicMilpScheduler::createPrecedenceConstraints;
        using DeterministicMilpScheduler::createSchedule;
        using DeterministicMilpScheduler::createTaskDurations;
        using DeterministicMilpScheduler::createTaskDurationsFirstIteration;
        using DeterministicMilpScheduler::createTaskDurationsOtherIterations;
        using DeterministicMilpScheduler::init;
    };

}  // namespace grstapse::mocks