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
#pragma once

// Global
#include <memory>
// External
#include <nlohmann/json.hpp>
// Local
#include "grstapse/geometric_planning/configuration_base.hpp"

namespace grstapse
{
    enum class GraphType
    {
        e_point,
        e_sampled_point,
        e_grid
    };
    NLOHMANN_JSON_SERIALIZE_ENUM(GraphType,
                                 {{GraphType::e_point, "point"},
                                  {GraphType::e_sampled_point, "sampled_point"},
                                  {GraphType::e_grid, "grid"}});

    /**!
     * Configuration from a graph
     */
    class GraphConfiguration : public ConfigurationBase
    {
       public:
        //! Deserializes from json
        static std::shared_ptr<const GraphConfiguration> deserializeFromJson(const nlohmann::json& j);

        //! \returns The id of the graph vertex
        [[nodiscard]] inline unsigned int id() const;

       protected:
        //! Default Constructor
        explicit GraphConfiguration(unsigned int id);

        unsigned int m_id;
    };

    // Inline Function
    unsigned int GraphConfiguration::id() const
    {
        return m_id;
    }

}  // namespace grstapse