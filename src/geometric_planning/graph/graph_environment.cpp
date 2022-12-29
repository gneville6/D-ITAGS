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
#include "grstapse/geometric_planning/graph/graph_environment.hpp"

// Local
#include "grstapse/common/utilities/constants.hpp"
#include "grstapse/common/utilities/custom_json_conversions.hpp"
#include "grstapse/common/utilities/error.hpp"
#include "grstapse/common/utilities/json_field_validator.hpp"
#include "grstapse/geometric_planning/graph/point/point_graph_environment.hpp"
#include "grstapse/geometric_planning/graph/point/sampled_point_graph_environment.hpp"

namespace grstapse
{
    std::shared_ptr<GraphEnvironment> GraphEnvironment::deserializeFromJson(const nlohmann::json& j)
    {
        validate(j, {{constants::k_graph_type, nlohmann::json::value_t::string}});
        GraphType graph_type = j[constants::k_graph_type].get<GraphType>();

        switch(graph_type)
        {
            case GraphType::e_point:
            {
                return j.get<std::shared_ptr<PointGraphEnvironment>>();
            }
            case GraphType::e_sampled_point:
            {
                return j.get<std::shared_ptr<SampledPointGraphEnvironment>>();
            }
            case GraphType::e_grid:
            {
                throw std::logic_error("Not Implemented");
            }
            default:
            {
                throw createLogicError("Unknown graph type");
            }
        }
    }

    GraphEnvironment::GraphEnvironment(GraphType graph_type)
        : EnvironmentBase(ConfigurationType::e_graph)
        , m_graph_type(graph_type)
    {}
}  // namespace grstapse