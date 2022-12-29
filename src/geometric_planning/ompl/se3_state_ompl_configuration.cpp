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
#include "grstapse/geometric_planning/ompl/se3_state_ompl_configuration.hpp"

// External
#include <ompl/base/ScopedState.h>
#include <ompl/base/goals/GoalState.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/spaces/SO3StateSpace.h>
// Local
#include "grstapse/common/utilities/constants.hpp"
#include "grstapse/common/utilities/json_field_validator.hpp"

namespace grstapse
{
    std::shared_ptr<const Se3StateOmplConfiguration> Se3StateOmplConfiguration::deserializeFromJson(
        const nlohmann::json& j)
    {
        validate(j,
                 {{constants::k_x, nlohmann::json::value_t::number_float},
                  {constants::k_y, nlohmann::json::value_t::number_float},
                  {constants::k_z, nlohmann::json::value_t::number_float},
                  {constants::k_qw, nlohmann::json::value_t::number_float},
                  {constants::k_qx, nlohmann::json::value_t::number_float},
                  {constants::k_qy, nlohmann::json::value_t::number_float},
                  {constants::k_qz, nlohmann::json::value_t::number_float}});

        auto rv = std::make_shared<Se3StateOmplConfiguration>();
        j.at(constants::k_x).get_to(rv->m_x);
        j.at(constants::k_y).get_to(rv->m_y);
        j.at(constants::k_z).get_to(rv->m_z);
        j.at(constants::k_qw).get_to(rv->m_qw);
        j.at(constants::k_qx).get_to(rv->m_qx);
        j.at(constants::k_qy).get_to(rv->m_qy);
        j.at(constants::k_qz).get_to(rv->m_qz);
        return rv;
    }

    Se3StateOmplConfiguration::Se3StateOmplConfiguration()
        : Se3OmplConfiguration(OmplGoalType::e_state)
        , m_x(0.0f)
        , m_y(0.0f)
        , m_z(0.0f)
        , m_qw(1.0f)
        , m_qx(0.0f)
        , m_qy(0.0f)
        , m_qz(0.0f)
    {}

    Se3StateOmplConfiguration::Se3StateOmplConfiguration(const float x, const float y, const float z)
        : Se3OmplConfiguration(OmplGoalType::e_state)
        , m_x(x)
        , m_y(y)
        , m_z(z)
        , m_qw(1.0f)
        , m_qx(0.0f)
        , m_qy(0.0f)
        , m_qz(0.0f)
    {}

    Se3StateOmplConfiguration::Se3StateOmplConfiguration(const float x,
                                                         const float y,
                                                         const float z,
                                                         const float qw,
                                                         const float qx,
                                                         const float qy,
                                                         const float qz)
        : Se3OmplConfiguration(OmplGoalType::e_state)
        , m_x(x)
        , m_y(y)
        , m_z(z)
        , m_qw(qw)
        , m_qx(qx)
        , m_qy(qy)
        , m_qz(qz)
    {}

    float Se3StateOmplConfiguration::euclideanDistance(
        const std::shared_ptr<const Se3StateOmplConfiguration>& rhs) const
    {
        const float x_diff = m_x - rhs->m_x;
        const float y_diff = m_y - rhs->m_y;
        const float z_diff = m_z - rhs->m_z;
        return std::sqrt(pow(x_diff, 2) + pow(y_diff, 2) + pow(z_diff, 2));
    }

    bool Se3StateOmplConfiguration::operator==(const Se3StateOmplConfiguration& rhs) const
    {
        return m_x == rhs.m_x && m_y == rhs.m_y && m_z == rhs.m_z && m_qw == rhs.m_qw && m_qx == rhs.m_qx &&
               m_qy == rhs.m_qy && m_qz == rhs.m_qz;
    }

    bool Se3StateOmplConfiguration::isEqual(const std::shared_ptr<const ConfigurationBase>& rhs) const
    {
        if(rhs->configurationType() != ConfigurationType::e_ompl)
        {
            return false;
        }

        const auto& rhs_ompl = std::dynamic_pointer_cast<const OmplConfiguration>(rhs);
        if(rhs_ompl->stateSpaceType() != OmplStateSpaceType::e_se3 || rhs_ompl->goalType() != OmplGoalType::e_state)
        {
            return false;
        }

        const auto& rhs_ompl_se3_state = std::dynamic_pointer_cast<const Se3StateOmplConfiguration>(rhs_ompl);

        return *this == *rhs_ompl_se3_state;
    }

    ompl::base::ScopedStatePtr Se3StateOmplConfiguration::convertToScopedStatePtr(
        const ompl::base::StateSpacePtr& state_space) const
    {
        auto rv     = std::make_shared<ompl::base::ScopedState<>>(state_space);
        auto* state = rv->get()->as<ompl::base::SE3StateSpace::StateType>();
        state->setX(m_x);
        state->setY(m_y);
        state->setZ(m_z);
        auto& so3_state = state->rotation();
        so3_state.w     = m_qw;
        so3_state.x     = m_qx;
        so3_state.y     = m_qy;
        so3_state.z     = m_qz;
        return rv;
    }

    ompl::base::GoalPtr Se3StateOmplConfiguration::convertToGoalPtr(
        const ompl::base::SpaceInformationPtr& space_information) const
    {
        auto rv     = std::make_shared<ompl::base::GoalState>(space_information);
        auto* state = space_information->allocState()->as<ompl::base::SE3StateSpace::StateType>();
        state->setX(m_x);
        state->setY(m_y);
        state->setZ(m_z);
        auto& so3_state = state->rotation();
        so3_state.w     = m_qw;
        so3_state.x     = m_qx;
        so3_state.y     = m_qy;
        so3_state.z     = m_qz;
        rv->setState(state);
        return rv;
    }

}  // namespace grstapse