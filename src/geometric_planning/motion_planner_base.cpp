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
#include "grstapse/geometric_planning/motion_planner_base.hpp"

// Local
#include "grstapse/common/utilities/constants.hpp"
#include "grstapse/common/utilities/timer_runner.hpp"
#include "grstapse/geometric_planning/configuration_base.hpp"
#include "grstapse/geometric_planning/motion_planning_query_result_base.hpp"

namespace grstapse
{
    unsigned int MotionPlannerBase::s_num_failures = 0;

    MotionPlannerBase::MotionPlannerBase(const std::shared_ptr<const MotionPlannerParametersBase>& parameters,
                                         const std::shared_ptr<EnvironmentBase>& environment)
        : m_parameters(parameters)
        , m_environment(environment)
    {}

    std::shared_ptr<const MotionPlanningQueryResultBase> MotionPlannerBase::query(
        const std::shared_ptr<const Species>& species,
        const std::shared_ptr<const ConfigurationBase>& initial_configuration,
        const std::shared_ptr<const ConfigurationBase>& goal_configuration)
    {
        TimerRunner timer_runner(constants::k_motion_planning_time);
        if(std::shared_ptr<const MotionPlanningQueryResultBase> result =
               getMemoized(species, initial_configuration, goal_configuration);
           result != nullptr)
        {
            return result;
        }

        // Compute and memoize
        std::shared_ptr<const MotionPlanningQueryResultBase> result =
            computeMotionPlan(species, initial_configuration, goal_configuration);
        auto iter = m_memoization.insert(
            std::pair(species, std::make_tuple(initial_configuration, goal_configuration, result)));
        return result;
    }

    float MotionPlannerBase::durationQuery(const std::shared_ptr<const Species>& species,
                                           const std::shared_ptr<const ConfigurationBase>& initial_configuration,
                                           const std::shared_ptr<const ConfigurationBase>& goal_configuration)
    {
        if(std::shared_ptr<const MotionPlanningQueryResultBase> result =
               query(species, initial_configuration, goal_configuration);
           result != nullptr)
        {
            return result->duration(species->speed());
        }
        return -1.0f;
    }

    std::shared_ptr<const MotionPlanningQueryResultBase> MotionPlannerBase::getMemoized(
        const std::shared_ptr<const Species>& species,
        const std::shared_ptr<const ConfigurationBase>& initial_configuration,
        const std::shared_ptr<const ConfigurationBase>& goal_configuration) const
    {
        if(!m_memoization.contains(species))
        {
            return nullptr;
        }

        auto range = m_memoization.equal_range(species);
        for(auto iter = range.first; iter != range.second; ++iter)
        {
            const MemoizationValue& mv = iter->second;
            if(!std::get<0>(mv)->isEqual(initial_configuration))
            {
                continue;
            }
            if(!std::get<1>(mv)->isEqual(goal_configuration))
            {
                continue;
            }
            return std::get<2>(mv);
        }

        return nullptr;
    }

    unsigned int MotionPlannerBase::numFailures()
    {
        return s_num_failures;
    }

    void MotionPlannerBase::clearCache()
    {
        m_memoization.clear();
    }
}  // namespace grstapse