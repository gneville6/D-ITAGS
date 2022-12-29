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
#include "grstapse/scheduling/milp/stochastic/scenario_milp_subscheduler.hpp"

// External
#include <fmt/format.h>
// Local
#include "grstapse/common/utilities/constants.hpp"
#include "grstapse/geometric_planning/graph/point/point_graph_configuration.hpp"
#include "grstapse/geometric_planning/graph/point/sampled_point_graph_motion_planner.hpp"
#include "grstapse/robot.hpp"
#include "grstapse/scheduling/scheduler_problem_inputs.hpp"
#include "grstapse/task.hpp"

namespace grstapse
{
    ScenarioMilpSubscheduler::ScenarioMilpSubscheduler(
        unsigned int index,
        const std::shared_ptr<const SchedulerProblemInputs>& problem_inputs,
        robin_hood::unordered_map<std::string, MutexConstraintInfo>& reduced_mutex_constraints)
        : DeterministicMilpSchedulerBase(problem_inputs, reduced_mutex_constraints)
        , m_index(index)
    {}

    std::string ScenarioMilpSubscheduler::createTaskStartName(unsigned int task_nr) const
    {
        return fmt::format("ts_{0:d}^{1:d}", task_nr, m_index);
    }

    std::string ScenarioMilpSubscheduler::createTaskFinishName(unsigned int task_nr) const
    {
        return fmt::format("tf_{0:d}^{1:d}", task_nr, m_index);
    }

    std::string ScenarioMilpSubscheduler::createDurationConstraintName(unsigned int task_nr) const
    {
        return fmt::format("dc_{0:d}^{1:d}", task_nr, m_index);
    }

    float ScenarioMilpSubscheduler::computeTaskDuration(unsigned int task_nr,
                                                        const std::vector<std::shared_ptr<const Robot>>& coalition)
    {
        if(coalition.empty())
        {
            return -1.0f;
        }

        std::shared_ptr<const Robot> widest_robot = nullptr;
        for(const std::shared_ptr<const Robot>& robot: coalition)
        {
            if(widest_robot == nullptr || robot->boundingRadius() > widest_robot->boundingRadius())
            {
                widest_robot = robot;
            }
        }
        auto motion_planner =
            std::dynamic_pointer_cast<SampledPointGraphMotionPlanner>(widest_robot->species()->motionPlanner());
        const std::shared_ptr<const Task>& task = m_problem_inputs->planTask(task_nr);
        return motion_planner->durationQuery(
            m_index,
            widest_robot->species(),
            std::dynamic_pointer_cast<const PointGraphConfiguration>(task->initialConfiguration()),
            std::dynamic_pointer_cast<const PointGraphConfiguration>(task->terminalConfiguration()));
    }

    std::string ScenarioMilpSubscheduler::createPrecedenceConstraintName(unsigned int i, unsigned int j) const
    {
        return fmt::format("tc_({0:d}, {1:d})^{2:d}", i, j, m_index);
    }

    void ScenarioMilpSubscheduler::addMutexConstraint(GRBModel& model,
                                                      const unsigned int i,
                                                      const double i_to_j_transition_duration,
                                                      const unsigned int j,
                                                      const double j_to_i_transition_duration)
    {
        const std::string p_ij_name = fmt::format("p_({0:d},{1:d})", i, j);
        GRBVar p_ij;
        if(m_index == 0)
        {
            p_ij = model.addVar(0.0, 1.0, 0.0, GRB_BINARY, p_ij_name);
            if(m_reduced_mutex_constraints.contains(p_ij_name))
            {
                m_reduced_mutex_constraints[p_ij_name].variable = p_ij;
            }
            else
            {
                m_reduced_mutex_constraints[p_ij_name] =
                    MutexConstraintInfo{.task_i = i, .task_j = j, .variable_name = p_ij_name, .variable = p_ij};
            }
        }
        else
        {
            p_ij = m_reduced_mutex_constraints[p_ij_name].variable;
        }

        // i -> j
        model.addGenConstrIndicator(
            p_ij,
            1,
            m_tasks_timepoints[j].start >= m_tasks_timepoints[i].finish + i_to_j_transition_duration,
            fmt::format("tc_({0:d},{1:d})^{2:d}", i, j, m_index));

        // j -> i
        model.addGenConstrIndicator(
            p_ij,
            0,
            m_tasks_timepoints[i].start >= m_tasks_timepoints[j].finish + j_to_i_transition_duration,
            fmt::format("tc_({0:d},{1:d})^{2:d}", j, i, m_index));
    }

    std::string ScenarioMilpSubscheduler::createInitialTransitionConstraintName(unsigned int task_nr) const
    {
        return fmt::format("itc_{0:d}^{1:d}", task_nr, m_index);
    }

    bool ScenarioMilpSubscheduler::isInitialTransitionMemoized(
        const std::shared_ptr<const ConfigurationBase>& configuration,
        const std::shared_ptr<const Robot>& robot) const
    {
        auto motion_planner =
            std::dynamic_pointer_cast<SampledPointGraphMotionPlanner>(robot->species()->motionPlanner());
        return motion_planner->isMemoized(
            m_index,
            robot->species(),
            std::dynamic_pointer_cast<const PointGraphConfiguration>(robot->initialConfiguration()),
            std::dynamic_pointer_cast<const PointGraphConfiguration>(configuration));
    }

    float ScenarioMilpSubscheduler::computeInitialTransitionDuration(
        const std::shared_ptr<const ConfigurationBase>& configuration,
        const std::shared_ptr<const Robot>& robot) const
    {
        auto motion_planner =
            std::dynamic_pointer_cast<SampledPointGraphMotionPlanner>(robot->species()->motionPlanner());
        return motion_planner->durationQuery(
            m_index,
            robot->species(),
            std::dynamic_pointer_cast<const PointGraphConfiguration>(robot->initialConfiguration()),
            std::dynamic_pointer_cast<const PointGraphConfiguration>(configuration));
    }

    bool ScenarioMilpSubscheduler::isTransitionMemoized(
        const std::shared_ptr<const ConfigurationBase>& initial_configuration,
        const std::shared_ptr<const ConfigurationBase>& goal_configuration,
        const std::shared_ptr<const Robot>& robot) const
    {
        auto motion_planner =
            std::dynamic_pointer_cast<SampledPointGraphMotionPlanner>(robot->species()->motionPlanner());
        return motion_planner->isMemoized(
            m_index,
            robot->species(),
            std::dynamic_pointer_cast<const PointGraphConfiguration>(initial_configuration),
            std::dynamic_pointer_cast<const PointGraphConfiguration>(goal_configuration));
    }

    float ScenarioMilpSubscheduler::computeTransitionDuration(
        const std::shared_ptr<const ConfigurationBase>& initial_configuration,
        const std::shared_ptr<const ConfigurationBase>& goal_configuration,
        const std::shared_ptr<const Robot>& robot) const
    {
        auto motion_planner =
            std::dynamic_pointer_cast<SampledPointGraphMotionPlanner>(robot->species()->motionPlanner());
        return motion_planner->durationQuery(
            m_index,
            robot->species(),
            std::dynamic_pointer_cast<const PointGraphConfiguration>(initial_configuration),
            std::dynamic_pointer_cast<const PointGraphConfiguration>(goal_configuration));
    }

    bool ScenarioMilpSubscheduler::createObjective(GRBModel& model)
    {
        GRBVar global_makespan   = model.getVarByName(constants::k_makespan);
        GRBVar scenario_makespan = model.addVar(0.0,
                                                GRB_INFINITY,
                                                0.0,
                                                GRB_CONTINUOUS,
                                                fmt::format("{0:s}_{1:d}", constants::k_makespan, m_index));
        GRBVar y_i               = model.addVar(0.0, 1.0, 0.0, GRB_BINARY, fmt::format("y_{0:d}", m_index));
        model.addGenConstrIndicator(y_i, 0, scenario_makespan <= global_makespan, fmt::format("yc_{0:d}", m_index));
        return true;
    }

    std::shared_ptr<const ScheduleBase> ScenarioMilpSubscheduler::computeSchedule()
    {
        // Intentional
        throw std::logic_error("Not implemented");
    }
}  // namespace grstapse