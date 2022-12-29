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
#include "grstapse/geometric_planning/ompl/se2_state_ompl_configuration.hpp"

// Global
#include <cmath>
// External
#include <ompl/base/ScopedState.h>
#include <ompl/base/goals/GoalState.h>
#include <ompl/base/spaces/SE2StateSpace.h>
// Local
#include "grstapse/common/utilities/constants.hpp"
#include "grstapse/common/utilities/json_field_validator.hpp"
#include "grstapse/geometric_planning/ompl/ompl_enums.hpp"

namespace grstapse
{
    Se2StateOmplConfiguration::Se2StateOmplConfiguration()
        : Se2OmplConfiguration(OmplGoalType::e_state)
        , m_x(0.0f)
        , m_y(0.0f)
        , m_yaw(0.0f)
    {}

    Se2StateOmplConfiguration::Se2StateOmplConfiguration(const float x, const float y, const float yaw)
        : Se2OmplConfiguration(OmplGoalType::e_state)
        , m_x(x)
        , m_y(y)
        , m_yaw(yaw)
    {}

    std::shared_ptr<const Se2StateOmplConfiguration> Se2StateOmplConfiguration::deserializeFromJson(
        const nlohmann::json& j)
    {
        validate(j,
                 {{constants::k_x, nlohmann::json::value_t::number_float},
                  {constants::k_y, nlohmann::json::value_t::number_float},
                  {constants::k_yaw, nlohmann::json::value_t::number_float}});

        auto rv = std::make_shared<Se2StateOmplConfiguration>();
        j.at(constants::k_x).get_to(rv->m_x);
        j.at(constants::k_y).get_to(rv->m_y);
        j.at(constants::k_yaw).get_to(rv->m_yaw);
        return rv;
    }

    float Se2StateOmplConfiguration::euclideanDistance(
        const std::shared_ptr<const Se2StateOmplConfiguration>& rhs) const
    {
        const float x_diff = m_x - rhs->m_x;
        const float y_diff = m_y - rhs->m_y;
        return std::sqrt(pow(x_diff, 2) + pow(y_diff, 2));
    }

    bool Se2StateOmplConfiguration::isEqual(const std::shared_ptr<const ConfigurationBase>& rhs) const
    {
        if(rhs->configurationType() != ConfigurationType::e_ompl)
        {
            return false;
        }

        const auto& rhs_ompl = std::dynamic_pointer_cast<const OmplConfiguration>(rhs);
        if(rhs_ompl->stateSpaceType() != OmplStateSpaceType::e_se2 || rhs_ompl->goalType() != OmplGoalType::e_state)
        {
            return false;
        }

        const auto& rhs_ompl_se2_state = std::dynamic_pointer_cast<const Se2StateOmplConfiguration>(rhs_ompl);

        return *this == *rhs_ompl_se2_state;
    }

    bool Se2StateOmplConfiguration::operator==(const Se2StateOmplConfiguration& rhs) const
    {
        return m_x == rhs.m_x && m_y == rhs.m_y && m_yaw == rhs.m_yaw;
    }

    ompl::base::ScopedStatePtr Se2StateOmplConfiguration::convertToScopedStatePtr(
        const ompl::base::StateSpacePtr& state_space) const
    {
        auto rv     = std::make_shared<ompl::base::ScopedState<>>(state_space);
        auto* state = rv->get()->as<ompl::base::SE2StateSpace::StateType>();
        state->setX(m_x);
        state->setY(m_y);
        state->setYaw(m_yaw);
        return rv;
    }

    ompl::base::GoalPtr Se2StateOmplConfiguration::convertToGoalPtr(
        const ompl::base::SpaceInformationPtr& space_information) const
    {
        auto rv     = std::make_shared<ompl::base::GoalState>(space_information);
        auto* state = space_information->allocState()->as<ompl::base::SE2StateSpace::StateType>();
        state->setX(m_x);
        state->setY(m_y);
        state->setYaw(m_yaw);
        rv->setState(state);
        return rv;
    }

}  // namespace grstapse