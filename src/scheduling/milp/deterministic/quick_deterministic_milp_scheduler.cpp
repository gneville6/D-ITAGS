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
#include "grstapse/scheduling/milp/deterministic/quick_deterministic_milp_scheduler.hpp"

// External
#include <fmt/format.h>
// Local
#include "grstapse/common/utilities/constants.hpp"
#include "grstapse/common/utilities/error.hpp"
#include "grstapse/geometric_planning/configuration_base.hpp"
#include "grstapse/geometric_planning/ompl/ompl_environment.hpp"
#include "grstapse/geometric_planning/ompl/ompl_motion_planner.hpp"
#include "grstapse/robot.hpp"
#include "grstapse/scheduling/milp/deterministic/deterministic_schedule.hpp"
#include "grstapse/scheduling/milp/milp_scheduler_parameters.hpp"
#include "grstapse/scheduling/scheduler_problem_inputs.hpp"
#include "grstapse/species.hpp"
#include "grstapse/task.hpp"

namespace grstapse
{
    QuickDeterministicMilpScheduler::QuickDeterministicMilpScheduler(
        const std::shared_ptr<const SchedulerProblemInputs>& problem_inputs)
        : DeterministicMilpScheduler(problem_inputs)
    {}

    std::shared_ptr<const ScheduleBase> QuickDeterministicMilpScheduler::computeSchedule()
    {
        auto milp_parameters =
            std::dynamic_pointer_cast<const MilpSchedulerParameters>(m_problem_inputs->schedulerParameters());
        if(milp_parameters->compute_transition_duration_heuristic)
        {
            if(!computeInitialTransitionHeuristicDurations() || !computeTransitionHeuristicDurations())
            {
                return nullptr;
            }
        }

        ++s_num_iterations;
        GRBModel model(s_environment);
        if(!createModel(model))
        {
            ++s_num_failures;
            return nullptr;
        }

        // Optimize model
        model.optimize();

        // Check status
        if(model.get(GRB_IntAttr_Status) != GRB_OPTIMAL)
        {
            ++s_num_failures;
            return nullptr;
        }

        return createSchedule(model);
    }

    std::shared_ptr<const DeterministicSchedule> QuickDeterministicMilpScheduler::createSchedule(GRBModel& model)
    {
        // Collect makespan
        const double makespan = model.getVarByName(constants::k_makespan).get(GRB_DoubleAttr_X);

        // Get original timepoints from the milp (these basically encode the precedence/resolved mutex constraints)
        std::vector<std::pair<float, float>> timepoints(m_tasks_timepoints.size());
        for(const TaskVariableInfo& task_timepoints: m_tasks_timepoints)
        {
            timepoints[task_timepoints.task_nr] =
                std::pair(task_timepoints.start.get(GRB_DoubleAttr_X), task_timepoints.finish.get(GRB_DoubleAttr_X));
        }

        // Sort by order of start
        std::sort(m_tasks_timepoints.begin(),
                  m_tasks_timepoints.end(),
                  [](const TaskVariableInfo& lhs, const TaskVariableInfo& rhs)
                  {
                      return lhs.start.get(GRB_DoubleAttr_X) < rhs.start.get(GRB_DoubleAttr_X);
                  });

        const unsigned int num_robots = m_problem_inputs->numberOfRobots();
        std::vector<int> previous_task(num_robots, -1);
        std::vector<std::shared_ptr<const ConfigurationBase>> previous_configurations;
        previous_configurations.reserve(num_robots);
        for(const std::shared_ptr<const Robot>& robot: m_problem_inputs->robots())
        {
            previous_configurations.push_back(robot->initialConfiguration());
        }

        for(TaskVariableInfo& task_timepoint: m_tasks_timepoints)
        {
            const std::shared_ptr<const Task>& task = m_problem_inputs->planTask(task_timepoint.task_nr);
            const std::shared_ptr<const ConfigurationBase>& initial_configuration  = task->initialConfiguration();
            const std::shared_ptr<const ConfigurationBase>& terminal_configuration = task->terminalConfiguration();
            for(unsigned int robot_nr: task_timepoint.coalition)
            {
                const bool first_task = previous_task[robot_nr] < 0;
                TaskTransitionInfo& task_transition_info =
                    previous_task[robot_nr] < 0 ? m_initial_transition_info[task_timepoint.task_nr]
                                                : m_transition_info[previous_task[robot_nr]][task_timepoint.task_nr];
                for(RobotTaskTransitionInfo& robot_task_transition_info: task_transition_info)
                {
                    if(robot_task_transition_info.robot_nr != robot_nr)
                    {
                        continue;
                    }

                    float previous_task_finish = 0.0f;
                    if(!first_task)
                    {
                        previous_task_finish = timepoints[previous_task[robot_nr]].second;
                    }

                    switch(robot_task_transition_info.computation_status)
                    {
                        case TransitionComputationStatus::e_none:
                        case TransitionComputationStatus::e_heuristic:
                        {
                            const float transition_duration =
                                m_problem_inputs->robot(robot_nr)->durationQuery(previous_configurations[robot_nr],
                                                                                 initial_configuration);
                            robot_task_transition_info.computation_status = TransitionComputationStatus::e_success;
                            robot_task_transition_info.duration           = transition_duration;

                            // If the task could not have started yet because of mp, delay the start
                            if(previous_task_finish + transition_duration > timepoints[task_timepoint.task_nr].first)
                            {
                                timepoints[task_timepoint.task_nr].first = previous_task_finish + transition_duration;
                                timepoints[task_timepoint.task_nr].second =
                                    timepoints[task_timepoint.task_nr].first + m_task_durations[task_timepoint.task_nr];
                            }

                            break;
                        }
                        case TransitionComputationStatus::e_failed:
                        {
                            throw createLogicError(
                                fmt::format("Request for failed transition (task nr: {0:d}; robot nr: {1:d})",
                                            task_timepoint.task_nr,
                                            robot_nr));
                        }
                        default:
                        {
                            // previously computed
                            break;
                        }
                    }
                }
            }
        }

        // Collect the precedence set mutex constraints
        std::vector<std::pair<unsigned int, unsigned int>> precedence_set_mutex_constraints;
        precedence_set_mutex_constraints.reserve(m_reduced_mutex_constraints.size());
        for(auto& [key, info]: m_reduced_mutex_constraints)
        {
            if(model.getVarByName(info.variable_name).get(GRB_DoubleAttr_X) > 0.5f)
            {
                precedence_set_mutex_constraints.emplace_back(info.task_i, info.task_j);
            }
            else
            {
                precedence_set_mutex_constraints.emplace_back(info.task_j, info.task_i);
            }
        }

        return std::make_shared<const DeterministicSchedule>(makespan, timepoints, precedence_set_mutex_constraints);
    }
}  // namespace grstapse