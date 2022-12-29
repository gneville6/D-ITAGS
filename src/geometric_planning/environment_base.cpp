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
#include "grstapse/geometric_planning/environment_base.hpp"

// Local
#include "grstapse/common/utilities/constants.hpp"
#include "grstapse/common/utilities/error.hpp"
#include "grstapse/common/utilities/json_field_validator.hpp"
#include "grstapse/geometric_planning/graph/graph_environment.hpp"
#include "grstapse/geometric_planning/ompl/ompl_environment.hpp"

namespace grstapse
{
    std::shared_ptr<EnvironmentBase> EnvironmentBase::deserializeFromJson(const nlohmann::json& j)
    {
        validate(j, {{constants::k_configuration_type, nlohmann::json::value_t::string}});
        const ConfigurationType configuration_type = j.at(constants::k_configuration_type).get<ConfigurationType>();
        switch(configuration_type)
        {
            case ConfigurationType::e_ompl:
            {
                return OmplEnvironment::deserializeFromJson(j);
            }
            case ConfigurationType::e_graph:
            {
                return GraphEnvironment::deserializeFromJson(j);
            }
            default:
            {
                throw createLogicError("Unknown task configuration type for loading environment");
            }
        }
    }

    EnvironmentBase::EnvironmentBase(ConfigurationType configuration_type)
        : m_configuration_type(configuration_type)
    {}
}  // namespace grstapse