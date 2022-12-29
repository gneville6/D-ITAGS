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
#include "grstapse/geometric_planning/configuration_base.hpp"

// Local
#include "grstapse/common/utilities/constants.hpp"
#include "grstapse/common/utilities/error.hpp"
#include "grstapse/geometric_planning/graph/graph_configuration.hpp"
#include "grstapse/geometric_planning/ompl/ompl_configuration.hpp"

namespace grstapse
{
    ConfigurationBase::ConfigurationBase(ConfigurationType type)
        : m_type(type)
    {}

    std::shared_ptr<const ConfigurationBase> ConfigurationBase::deserializeFromJson(const nlohmann::json& j)
    {
        const ConfigurationType type = j.at(constants::k_configuration_type).get<ConfigurationType>();
        switch(type)
        {
            case ConfigurationType::e_ompl:
            {
                return OmplConfiguration::deserializeFromJson(j);
            }
            case ConfigurationType::e_graph:
            {
                return GraphConfiguration::deserializeFromJson(j);
            }
            default:
            {
                throw createLogicError("Unknown configuration type");
            }
        }
    }
}  // namespace grstapse