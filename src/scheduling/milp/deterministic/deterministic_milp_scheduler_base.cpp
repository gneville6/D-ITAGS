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
#include "grstapse/scheduling/milp/deterministic/deterministic_milp_scheduler_base.hpp"

// External
#include <fmt/format.h>
// Local
#include "grstapse/common/utilities/logger.hpp"
#include "grstapse/geometric_planning/configuration_base.hpp"
#include "grstapse/robot.hpp"
#include "grstapse/scheduling/scheduler_problem_inputs.hpp"
#include "grstapse/task.hpp"

namespace grstapse
{
    DeterministicMilpSchedulerBase::DeterministicMilpSchedulerBase(
        const std::shared_ptr<const SchedulerProblemInputs>& problem_inputs,
        robin_hood::unordered_map<std::string, MutexConstraintInfo>& reduced_mutex_constraints)
        : MilpSchedulerBase(problem_inputs)
        , m_reduced_mutex_constraints(reduced_mutex_constraints)
    {
        init();
    }

    void DeterministicMilpSchedulerBase::init()
    {
        const unsigned int num_tasks      = m_problem_inputs->numberOfPlanTasks();
        const unsigned int num_robots     = m_problem_inputs->numberOfRobots();
        const Eigen::MatrixXf& allocation = m_problem_inputs->allocation();

        m_task_finishes = std::unique_ptr<GRBVar>(new GRBVar[num_tasks]);
        m_tasks_timepoints.reserve(num_tasks);
        m_task_durations.resize(num_tasks, -1.0);

        std::vector<TaskTransitionInfo> inner_vec;
        TaskTransitionInfo transition_info;

        // Fill transition duration data structures
        m_initial_transition_info.reserve(num_tasks);
        m_transition_info.reserve(num_tasks);
        for(unsigned int task_i = 0; task_i < num_tasks; ++task_i)
        {
            inner_vec.resize(0);
            inner_vec.reserve(num_tasks);
            for(unsigned int task_j = 0; task_j < num_tasks; ++task_j)
            {
                transition_info.resize(0);
                // Ignore self transitions
                if(task_i != task_j)
                {
                    for(unsigned int robot = 0; robot < num_robots; ++robot)
                    {
                        if(allocation(task_i, robot) && allocation(task_j, robot))
                        {
                            transition_info.push_back(
                                RobotTaskTransitionInfo{.robot_nr           = robot,
                                                        .computation_status = TransitionComputationStatus::e_none,
                                                        .duration           = -1.0});
                        }
                    }
                }
                inner_vec.emplace_back(transition_info);
            }
            m_transition_info.emplace_back(inner_vec);

            transition_info.resize(0);
            for(unsigned int robot = 0; robot < num_robots; ++robot)
            {
                if(allocation(task_i, robot))
                {
                    transition_info.push_back(
                        RobotTaskTransitionInfo{.robot_nr           = robot,
                                                .computation_status = TransitionComputationStatus::e_none,
                                                .duration           = -1.0});
                }
            }
            m_initial_transition_info.push_back(transition_info);
        }
    }

    // region createTaskDurations
    bool DeterministicMilpSchedulerBase::createTaskDurations(GRBModel& model)
    {
        // First iteration
        if(m_tasks_timepoints.empty())
        {
            return createTaskDurationsFirstIteration(model);
        }

        return createTaskDurationsOtherIterations(model);
    }

    bool DeterministicMilpSchedulerBase::createTaskDurationsFirstIteration(GRBModel& model)
    {
        // Allocation matrix is M X N (number_of_tasks X number_of_robots)
        const unsigned int num_tasks      = m_problem_inputs->numberOfPlanTasks();
        const unsigned int num_robots     = m_problem_inputs->numberOfRobots();
        const Eigen::MatrixXf& allocation = m_problem_inputs->allocation();

        std::vector<std::shared_ptr<const Robot>> coalition;
        std::vector<unsigned int> coalition_numbers;

        // Compute coalitions and task durations
        for(unsigned int task_nr = 0; task_nr < num_tasks; ++task_nr)
        {
            coalition.resize(0);
            coalition_numbers.resize(0);
            for(unsigned int robot_nr = 0; robot_nr < num_robots; ++robot_nr)
            {
                if(allocation(task_nr, robot_nr))
                {
                    coalition.push_back(m_problem_inputs->robot(robot_nr));
                    coalition_numbers.push_back(robot_nr);
                }
            }

            m_task_durations[task_nr] = computeTaskDuration(task_nr, coalition);

            // At least one motion plan cannot be solved
            if(m_task_durations[task_nr] < 0.0)
            {
                return false;
            }

            // Create MILP variables and constraints
            GRBVar start  = model.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS, createTaskStartName(task_nr));
            GRBVar finish = model.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS, createTaskFinishName(task_nr));

            m_tasks_timepoints.push_back(
                TaskVariableInfo{.task_nr = task_nr, .start = start, .finish = finish, .coalition = coalition_numbers});
            m_task_finishes.get()[task_nr] = finish;  //!< For makespan
            model.addConstr(
                m_tasks_timepoints[task_nr].finish - m_tasks_timepoints[task_nr].start == m_task_durations[task_nr],
                createDurationConstraintName(task_nr));
        }

        return true;
    }

    bool DeterministicMilpSchedulerBase::createTaskDurationsOtherIterations(GRBModel& model)
    {
        // TODO(Andrew): warm start. probably need to grab values before previous model is destroyed
        const unsigned int num_tasks = m_problem_inputs->numberOfPlanTasks();
        for(unsigned int task_nr = 0; task_nr < num_tasks; ++task_nr)
        {
            m_tasks_timepoints[task_nr].start =
                model.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS, createTaskStartName(task_nr));
            m_tasks_timepoints[task_nr].finish =
                model.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS, createTaskFinishName(task_nr));
            m_task_finishes.get()[task_nr] = m_tasks_timepoints[task_nr].finish;  //!< For makespan

            model.addConstr(
                m_tasks_timepoints[task_nr].finish - m_tasks_timepoints[task_nr].start == m_task_durations[task_nr],
                createDurationConstraintName(task_nr));
        }

        return true;
    }
    // endregion

    bool DeterministicMilpSchedulerBase::createPrecedenceConstraints(GRBModel& model)
    {
        for(const auto& [predecessor, successor]: m_problem_inputs->precedenceConstraints())
        {
            auto [mp_failure, transition_duration] = checkTransitionFeasibility(predecessor, successor);
            if(mp_failure)
            {
#ifdef DEBUG
                Logger::debug("Scheduling failure: A motion plan from '{0:s}' to '{1:s}' cannot not be computed",
                              m_problem_inputs->planTask(predecessor)->name(),
                              m_problem_inputs->planTask(successor)->name());
#endif
                return false;
            }

            // Note: If there is no transition between task i and j (no single robot is assigned to both then the
            // transition_duration is 0 and the following equation still holds
            model.addConstr(
                m_tasks_timepoints[successor].start - m_tasks_timepoints[predecessor].finish >= transition_duration,
                fmt::format("pc_({0:d},{1:d})", predecessor, successor));
        }

        if(!m_mp_induced_precedence_constraints.empty())
        {
            for(const auto& [predecessor, successor]: m_mp_induced_precedence_constraints)
            {
                auto [mp_failure, transition_duration] = checkTransitionFeasibility(predecessor, successor);
                if(mp_failure)
                {
#ifdef DEBUG
                    Logger::debug("Scheduling failure: A motion plan from '{0:s}' to '{1:s}' cannot not be computed",
                                  m_problem_inputs->planTask(predecessor)->name(),
                                  m_problem_inputs->planTask(successor)->name());
#endif
                    return false;
                }

                model.addConstr(
                    m_tasks_timepoints[successor].start - m_tasks_timepoints[predecessor].finish >= transition_duration,
                    createPrecedenceConstraintName(predecessor, successor));
            }
        }

        return true;
    }

    // region createMutexConstraints
    bool DeterministicMilpSchedulerBase::createMutexConstraints(GRBModel& model)
    {
        // First iteration
        if(m_reduced_mutex_constraints.empty())
        {
            return createMutexConstraintsFirstIteration(model);
        }

        return createMutexConstraintsOtherIterations(model);
    }

    bool DeterministicMilpSchedulerBase::createMutexConstraintsFirstIteration(GRBModel& model)
    {
        const std::multimap<unsigned int, unsigned int>& precedence_constraints =
            m_problem_inputs->precedenceConstraints();
        for(const auto& [task_i, task_j]: m_problem_inputs->mutexConstraints())
        {
            bool precedence_set = false;

            // Find precedence from task_i -> task_j
            auto iterator_bounds = precedence_constraints.equal_range(task_i);
            for(auto i = iterator_bounds.first; i != iterator_bounds.second; ++i)
            {
                if(i->second == task_j)
                {
                    precedence_set = true;
                    break;
                }
            }
            if(precedence_set)
            {
                continue;
            }

            // Find precedence from task_j -> task_i
            iterator_bounds = precedence_constraints.equal_range(task_j);
            for(auto i = iterator_bounds.first; i != iterator_bounds.second; ++i)
            {
                if(i->second == task_i)
                {
                    precedence_set = true;
                    break;
                }
            }
            if(precedence_set)
            {
                continue;
            }

            // Check if transition from task i to j is possible
            auto [i_to_j_mp_failure, i_to_j_transition_duration] = checkTransitionFeasibility(task_i, task_j);
            // Check if transition from task j to i is possible
            auto [j_to_i_mp_failure, j_to_i_transition_duration] = checkTransitionFeasibility(task_j, task_i);

            // If neither is possible then one of the robots that is allocated to both currently, cannot be
            if(i_to_j_mp_failure && j_to_i_mp_failure)
            {
                return false;
            }
            else if(i_to_j_mp_failure)  // j -> i precedence is forced by mp
            {
                model.addConstr(m_tasks_timepoints[task_i].start >=
                                m_tasks_timepoints[task_j].finish + j_to_i_transition_duration);
                m_mp_induced_precedence_constraints.insert(std::pair(task_i, task_j));
                continue;
            }
            else if(j_to_i_mp_failure)  // i -> j precedence is forced by mp
            {
                model.addConstr(m_tasks_timepoints[task_j].start >=
                                m_tasks_timepoints[task_i].finish + i_to_j_transition_duration);
                m_mp_induced_precedence_constraints.insert(std::pair(task_j, task_i));
                continue;
            }

            // Add Mutex Constraint
            addMutexConstraint(model, task_i, i_to_j_transition_duration, task_j, j_to_i_transition_duration);
        }
        return true;
    }

    bool DeterministicMilpSchedulerBase::createMutexConstraintsOtherIterations(GRBModel& model)
    {
        auto tmp_reduced_mutex_constraints = m_reduced_mutex_constraints;
        for(auto& [key, info]: m_reduced_mutex_constraints)
        {
            auto [i_to_j_mp_failure, i_to_j_transition_duration] = checkTransitionFeasibility(info.task_i, info.task_j);
            auto [j_to_i_mp_failure, j_to_i_transition_duration] = checkTransitionFeasibility(info.task_j, info.task_i);

            // One of the robots allocated to tasks i and j can not transition between them
            if(i_to_j_mp_failure && j_to_i_mp_failure)
            {
                return false;
            }
            else if(i_to_j_mp_failure)
            {
                m_mp_induced_precedence_constraints.insert(std::pair(info.task_j, info.task_i));
                tmp_reduced_mutex_constraints.erase(info.variable_name);
                model.addConstr(m_tasks_timepoints[info.task_i].start >=
                                m_tasks_timepoints[info.task_j].finish + j_to_i_transition_duration);
                continue;
            }
            else if(j_to_i_mp_failure)
            {
                m_mp_induced_precedence_constraints.insert(std::pair(info.task_i, info.task_j));
                tmp_reduced_mutex_constraints.erase(info.variable_name);
                model.addConstr(m_tasks_timepoints[info.task_j].start >=
                                m_tasks_timepoints[info.task_i].finish + i_to_j_transition_duration);
                continue;
            }

            // Add Mutex Constraint
            addMutexConstraint(model, info.task_i, i_to_j_transition_duration, info.task_j, j_to_i_transition_duration);
        }
        m_reduced_mutex_constraints = tmp_reduced_mutex_constraints;
        return true;
    }
    // endregion

    bool DeterministicMilpSchedulerBase::createInitialTransitions(GRBModel& model)
    {
        const unsigned int num_tasks = m_problem_inputs->numberOfPlanTasks();

        for(unsigned int task_nr = 0; task_nr < num_tasks; ++task_nr)
        {
            const TaskTransitionInfo& task_transition_info = m_initial_transition_info[task_nr];
            float earliest_start                           = 0.0;
            for(const RobotTaskTransitionInfo& robot_task_transition_info: task_transition_info)
            {
                switch(robot_task_transition_info.computation_status)
                {
                    case TransitionComputationStatus::e_heuristic:
                        [[fallthrough]];
                    case TransitionComputationStatus::e_success:
                    {
                        earliest_start = std::max(earliest_start, robot_task_transition_info.duration);
                        break;
                    }
                    default:
                    {
                        break;
                    }
                }
            }
            model.addConstr(m_tasks_timepoints[task_nr].start >= earliest_start,
                            createInitialTransitionConstraintName(task_nr));
        }
        return true;
    }

    bool DeterministicMilpSchedulerBase::computeInitialTransitionHeuristicDurations()
    {
        const unsigned int num_tasks = m_problem_inputs->numberOfPlanTasks();
        for(unsigned int task_nr = 0; task_nr < num_tasks; ++task_nr)
        {
            TaskTransitionInfo& task_transition_info = m_initial_transition_info[task_nr];
            const std::shared_ptr<const ConfigurationBase>& task_initial_configuration =
                m_problem_inputs->planTask(task_nr)->initialConfiguration();

            for(RobotTaskTransitionInfo& robot_task_transition_info: task_transition_info)
            {
                const std::shared_ptr<const Robot>& robot =
                    m_problem_inputs->robot(robot_task_transition_info.robot_nr);
                bool is_memoized = isInitialTransitionMemoized(task_initial_configuration, robot);

                if(is_memoized)
                {
                    const float duration = computeInitialTransitionDuration(task_initial_configuration, robot);

                    // Transition fails
                    if(duration < 0.0)
                    {
                        robot_task_transition_info.computation_status = TransitionComputationStatus::e_failed;
                        return false;
                    }

                    // Transition succeeds
                    robot_task_transition_info.computation_status = TransitionComputationStatus::e_success;
                    robot_task_transition_info.duration           = duration;
                }
                else
                {
                    const std::shared_ptr<const ConfigurationBase>& initial_configuration =
                        robot->initialConfiguration();
                    robot_task_transition_info.computation_status = TransitionComputationStatus::e_heuristic;
                    robot_task_transition_info.duration =
                        computeInitialTransitionHeuristicDuration(task_initial_configuration, robot);
                }
            }
        }
        return true;
    }

    float DeterministicMilpSchedulerBase::computeInitialTransitionHeuristicDuration(
        const std::shared_ptr<const ConfigurationBase>& configuration,
        const std::shared_ptr<const Robot>& robot) const
    {
        return robot->initialConfiguration()->euclideanDistance(configuration) / robot->speed();
    }

    bool DeterministicMilpSchedulerBase::computeTransitionHeuristicDurations()
    {
        const unsigned int num_tasks  = m_problem_inputs->numberOfPlanTasks();
        const unsigned int num_robots = m_problem_inputs->numberOfRobots();

        for(unsigned int task_i = 0; task_i < num_tasks; ++task_i)
        {
            for(unsigned int task_j = 0; task_j < num_tasks; ++task_j)
            {
                if(task_i == task_j)
                {
                    continue;
                }

                const std::shared_ptr<const ConfigurationBase>& task_i_terminal_configuration =
                    m_problem_inputs->planTask(task_i)->terminalConfiguration();
                const std::shared_ptr<const ConfigurationBase>& task_j_initial_configuration =
                    m_problem_inputs->planTask(task_j)->initialConfiguration();

                for(RobotTaskTransitionInfo& robot_task_transition_info: m_transition_info[task_i][task_j])
                {
                    const std::shared_ptr<const Robot>& robot =
                        m_problem_inputs->robot(robot_task_transition_info.robot_nr);
                    bool is_memoized =
                        isTransitionMemoized(task_i_terminal_configuration, task_j_initial_configuration, robot);

                    if(is_memoized)
                    {
                        const float duration = computeTransitionDuration(task_i_terminal_configuration,
                                                                         task_j_initial_configuration,
                                                                         robot);

                        // Transition fails
                        if(duration < 0.0)
                        {
                            robot_task_transition_info.computation_status = TransitionComputationStatus::e_failed;
                            return false;
                        }

                        // Transition succeeds
                        robot_task_transition_info.computation_status = TransitionComputationStatus::e_success;
                        robot_task_transition_info.duration           = duration;
                    }
                    else
                    {
                        const float duration = computeTransitionHeuristicDuration(task_i_terminal_configuration,
                                                                                  task_j_initial_configuration,
                                                                                  robot);
                        robot_task_transition_info.computation_status = TransitionComputationStatus::e_heuristic;
                        robot_task_transition_info.duration           = duration;
                    }
                }
            }
        }
        return true;
    }

    float DeterministicMilpSchedulerBase::computeTransitionHeuristicDuration(
        const std::shared_ptr<const ConfigurationBase>& initial_configuration,
        const std::shared_ptr<const ConfigurationBase>& goal_configuration,
        const std::shared_ptr<const Robot>& robot) const
    {
        return initial_configuration->euclideanDistance(goal_configuration) / robot->speed();
    }

    std::pair<bool, float> DeterministicMilpSchedulerBase::checkTransitionFeasibility(const unsigned int i,
                                                                                      const unsigned int j)
    {
        float duration = 0.0;
        for(const RobotTaskTransitionInfo& robot_task_transition_info: m_transition_info[i][j])
        {
            switch(robot_task_transition_info.computation_status)
            {
                case TransitionComputationStatus::e_failed:
                {
                    return std::pair(true, -1.0);
                }
                case TransitionComputationStatus::e_heuristic:
                    [[fallthrough]];
                case TransitionComputationStatus::e_success:
                {
                    duration = std::max(duration, robot_task_transition_info.duration);
                    break;
                }
                default:
                {
                    break;
                }
            }
        }
        return std::pair(false, duration);
    }

}  // namespace grstapse