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
#include "grstapse/geometric_planning/ompl/se2_ompl_configuration.hpp"

#include <ompl/base/ScopedState.h>

// Local
#include "grstapse/common/utilities/constants.hpp"
#include "grstapse/common/utilities/error.hpp"
#include "grstapse/common/utilities/json_field_validator.hpp"
#include "grstapse/geometric_planning/ompl/se2_state_ompl_configuration.hpp"

namespace grstapse
{
    Se2OmplConfiguration::Se2OmplConfiguration(OmplGoalType goal_type)
        : OmplConfiguration(goal_type, OmplStateSpaceType::e_se2)
    {}

    std::shared_ptr<const Se2OmplConfiguration> Se2OmplConfiguration::deserializeFromJson(const nlohmann::json& j)
    {
        validate(j, {{constants::k_goal_type, nlohmann::json::value_t::string}});
        OmplGoalType type = j.at(constants::k_goal_type).get<OmplGoalType>();
        switch(type)
        {
            case OmplGoalType::e_state:
            {
                return Se2StateOmplConfiguration::deserializeFromJson(j);
            }
            case OmplGoalType::e_set_of_states:
            {
                throw createLogicError("Not implemented");
            }
            case OmplGoalType::e_space:
            {
                throw createLogicError("Not implemented");
            }
            default:
            {
                throw createLogicError("Unknown goal type");
            }
        }
    }

    float Se2OmplConfiguration::euclideanDistance(const std::shared_ptr<const ConfigurationBase>& rhs) const
    {
        if(rhs->configurationType() != ConfigurationType::e_ompl)
        {
            throw createLogicError("Differing TaskConfigurationTypes");
        }

        const auto& rhs_ompl = std::dynamic_pointer_cast<const OmplConfiguration>(rhs);
        if(rhs_ompl->stateSpaceType() != OmplStateSpaceType::e_se2)
        {
            throw createLogicError("Differing OmplStateSpaceTypes");
        }

        switch(rhs_ompl->goalType())
        {
            case OmplGoalType::e_state:
            {
                const auto& rhs_ompl_se2_state = std::dynamic_pointer_cast<const Se2StateOmplConfiguration>(rhs_ompl);
                return euclideanDistance(rhs_ompl_se2_state);
            }
            case OmplGoalType::e_set_of_states:
            {
                throw createLogicError("Not Implemented");
            }
            case OmplGoalType::e_space:
            {
                throw createLogicError("Not Implemented");
            }
            default:
            {
                throw createLogicError("Unknown OmplGoalType");
            }
        }
    }
}  // namespace grstapse