/*
 * Graphically Recursive Simultaneous Task Allocation, Planning,
 * Scheduling, and Execution
 *
 * Copyright (C) 2020-2021
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
#include "grstapse/task.hpp"

// Local
#include "grstapse/geometric_planning/motion_planning_query_result_base.hpp"
#include "grstapse/robot.hpp"
#include "grstapse/task_planning/sas/sas_action.hpp"

namespace grstapse
{
    Task::Task(const std::shared_ptr<SasAction>& symbolic_action,
               const Eigen::VectorXf& desired_traits,
               const std::shared_ptr<const ConfigurationBase>& initial_configuration,
               const std::shared_ptr<const ConfigurationBase>& terminal_configuration)
        : m_symbolic_action(symbolic_action)
        , m_desired_traits(desired_traits)
        , m_initial_configuration(initial_configuration)
        , m_terminal_configuration(terminal_configuration)
    {}

    const std::string& Task::name() const
    {
        return m_symbolic_action->name();
    }

    float Task::staticDuration() const
    {
        return m_symbolic_action->duration();
    }

    std::shared_ptr<const MotionPlanningQueryResultBase> Task::motionPlanningQuery(
        const std::vector<std::shared_ptr<const Robot>>& coalition) const
    {
        if(coalition.empty())
        {
            return nullptr;
        }

        std::shared_ptr<const Robot> widest_robot = nullptr;
        for(const std::shared_ptr<const Robot>& robot: coalition)
        {
            if(widest_robot == nullptr || robot->boundingRadius() > widest_robot->boundingRadius())
            {
                widest_robot = robot;
            }
        }
        return widest_robot->motionPlanningQuery(m_initial_configuration, m_terminal_configuration);
    }

    float Task::computeDuration(const std::vector<std::shared_ptr<const Robot>>& coalition) const
    {
        if(coalition.empty())
        {
            return m_symbolic_action->duration();
        }

        // Get the route for the widest robot
        const std::shared_ptr<const MotionPlanningQueryResultBase> motion_planning_result =
            motionPlanningQuery(coalition);

        if(motion_planning_result == nullptr || motion_planning_result->status() != MotionPlannerQueryStatus::e_success)
        {
            return -1.0f;
        }

        // Get the slowest speed
        float slowest_speed = -1.0f;
        for(const std::shared_ptr<const Robot>& robot: coalition)
        {
            slowest_speed = std::max(slowest_speed, robot->speed());
        }
        return motion_planning_result->length() / slowest_speed + m_symbolic_action->duration();
    }
}  // namespace grstapse