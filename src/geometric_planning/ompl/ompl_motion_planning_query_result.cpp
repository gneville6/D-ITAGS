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
#include "grstapse/geometric_planning/ompl/ompl_motion_planning_query_result.hpp"

// External
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/geometric/PathGeometric.h>
// Local
#include "grstapse/common/utilities/constants.hpp"
#include "grstapse/common/utilities/error.hpp"
#include "grstapse/geometric_planning/configuration_base.hpp"
#include "grstapse/geometric_planning/ompl/ompl_enums.hpp"

namespace grstapse
{
    OmplMotionPlanningQueryResult::OmplMotionPlanningQueryResult(
        MotionPlannerQueryStatus status,
        const std::shared_ptr<const ompl::geometric::PathGeometric>& path)
        : MotionPlanningQueryResultBase(status)
        , m_path(path)
    {}

    float OmplMotionPlanningQueryResult::length() const
    {
        if(m_path == nullptr)
        {
            return -1.0f;
        }
        return m_path->length();
    }

    void OmplMotionPlanningQueryResult::serializeToJson(nlohmann::json& j) const
    {
        if(m_path == nullptr)
        {
            j = nullptr;
        }

        for(unsigned int i = 0, end = m_path->getStateCount(); i < end; ++i)
        {
            nlohmann::json waypoint_j;
            const ompl::base::State* state = m_path->getState(i);

            const auto se2_state = state->as<ompl::base::SE2StateSpace::StateType>();
            if(!se2_state)
            {
                throw createLogicError("Currently we only handle SE2 waypoints");
            }
            waypoint_j[constants::k_configuration_type] = ConfigurationType::e_ompl;
            waypoint_j[constants::k_goal_type]          = OmplGoalType::e_state;
            waypoint_j[constants::k_state_space_type]   = OmplStateSpaceType::e_se2;
            waypoint_j[constants::k_x]                  = se2_state->getX();
            waypoint_j[constants::k_y]                  = se2_state->getY();
            waypoint_j[constants::k_yaw]                = se2_state->getYaw();
            j.push_back(waypoint_j);
        }
    }

}  // namespace grstapse