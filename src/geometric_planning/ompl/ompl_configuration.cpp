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
#include "grstapse/geometric_planning/ompl/ompl_configuration.hpp"

// Local
#include "grstapse/common/utilities/constants.hpp"
#include "grstapse/common/utilities/error.hpp"
#include "grstapse/common/utilities/json_field_validator.hpp"
#include "grstapse/geometric_planning/ompl/se2_ompl_configuration.hpp"
#include "grstapse/geometric_planning/ompl/se3_ompl_configuration.hpp"

namespace grstapse
{
    std::shared_ptr<const OmplConfiguration> OmplConfiguration::deserializeFromJson(const nlohmann::json& j)
    {
        validate(j, {{constants::k_state_space_type, nlohmann::json::value_t::string}});
        OmplStateSpaceType type = j.at(constants::k_state_space_type).get<OmplStateSpaceType>();

        switch(type)
        {
            case OmplStateSpaceType::e_se2:
            {
                return Se2OmplConfiguration::deserializeFromJson(j);
            }
            case OmplStateSpaceType::e_se3:
            {
                return Se3OmplConfiguration::deserializeFromJson(j);
            }
            default:
            {
                throw createLogicError("Unknown state space type");
            }
        }
    }
    OmplConfiguration::OmplConfiguration(OmplGoalType goal_type, OmplStateSpaceType state_space_type)
        : ConfigurationBase(ConfigurationType::e_ompl)
        , m_goal_type(goal_type)
        , m_state_space_type(state_space_type)
    {}
}  // namespace grstapse