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
#include "grstapse/geometric_planning/graph/point/sampled_point_graph_motion_planner.hpp"

// Local
#include "grstapse/geometric_planning/graph/point/point_graph_motion_planner.hpp"
#include "grstapse/geometric_planning/graph/point/sampled_point_graph_environment.hpp"

namespace grstapse
{
    SampledPointGraphMotionPlanner::SampledPointGraphMotionPlanner(
        const std::shared_ptr<const MotionPlannerParametersBase>& parameters,
        const std::shared_ptr<SampledPointGraphEnvironment>& environment)
        : GraphMotionPlannerBase(parameters, environment)
    {
        m_sub_motion_planners.reserve(environment->numGraphs());
        for(unsigned int i = 0, end = environment->numGraphs(); i < end; ++i)
        {
            m_sub_motion_planners.push_back(
                std::make_shared<PointGraphMotionPlanner>(parameters, environment->graph(i)));
        }
    }

    std::shared_ptr<const MotionPlanningQueryResultBase> SampledPointGraphMotionPlanner::query(
        unsigned int index,
        const std::shared_ptr<const Species>& species,
        const std::shared_ptr<const PointGraphConfiguration>& initial_configuration,
        const std::shared_ptr<const PointGraphConfiguration>& goal_configuration)
    {
        assert(index < std::dynamic_pointer_cast<SampledPointGraphEnvironment>(m_environment)->numGraphs());
        return m_sub_motion_planners[index]->query(species, initial_configuration, goal_configuration);
    }

    bool SampledPointGraphMotionPlanner::isMemoized(
        unsigned int index,
        const std::shared_ptr<const Species>& species,
        const std::shared_ptr<const PointGraphConfiguration>& initial_configuration,
        const std::shared_ptr<const PointGraphConfiguration>& goal_configuration) const
    {
        assert(index < std::dynamic_pointer_cast<SampledPointGraphEnvironment>(m_environment)->numGraphs());
        return m_sub_motion_planners[index]->isMemoized(species, initial_configuration, goal_configuration);
    }

    float SampledPointGraphMotionPlanner::durationQuery(
        unsigned int index,
        const std::shared_ptr<const Species>& species,
        const std::shared_ptr<const PointGraphConfiguration>& initial_configuration,
        const std::shared_ptr<const PointGraphConfiguration>& goal_configuration)
    {
        assert(index < std::dynamic_pointer_cast<SampledPointGraphEnvironment>(m_environment)->numGraphs());
        return m_sub_motion_planners[index]->durationQuery(species, initial_configuration, goal_configuration);
    }

    std::shared_ptr<const MotionPlanningQueryResultBase> SampledPointGraphMotionPlanner::computeMotionPlan(
        const std::shared_ptr<const Species>& species,
        const std::shared_ptr<const ConfigurationBase>& initial_configuration,
        const std::shared_ptr<const ConfigurationBase>& goal_configuration)
    {
        // This is intentional
        throw std::logic_error("Not implemented");
    }

}  // namespace grstapse