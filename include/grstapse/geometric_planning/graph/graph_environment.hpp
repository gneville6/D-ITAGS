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

// Local
#include "grstapse/geometric_planning/environment_base.hpp"
#include "grstapse/geometric_planning/graph/graph_configuration.hpp"

namespace grstapse
{
    /**!
     *  A base class for graph based environments for motion planning
     */
    class GraphEnvironment : public EnvironmentBase
    {
       public:
        //! Load an environment from one or more files (filepaths are encoded in json)
        [[nodiscard]] static std::shared_ptr<GraphEnvironment> deserializeFromJson(const nlohmann::json& j);

       protected:
        //! Constructor
        explicit GraphEnvironment(GraphType graph_type);

        GraphType m_graph_type;
    };

}  // namespace grstapse