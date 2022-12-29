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
#include "grstapse/common/search/undirected_graph/undirected_graph.hpp"
#include "grstapse/geometric_planning/graph/graph_environment.hpp"
#include "grstapse/geometric_planning/graph/point/point_graph_configuration.hpp"

#include "sampled_point_graph_environment.hpp"

namespace grstapse
{
    /**!
     * An environment for an undirected graph were each vertex is point in 2D space
     */
    class PointGraphEnvironment
        : public UndirectedGraph<PointGraphConfiguration>
        , public GraphEnvironment
    {
       public:
        //! Constructor
        PointGraphEnvironment();

        [[nodiscard]] std::shared_ptr<PointGraphEnvironment> shallowCopy() const;

        //!
        [[nodiscard]] std::shared_ptr<UndirectedGraph<PointGraphConfiguration>::Vertex> findVertex(
            const std::shared_ptr<const PointGraphConfiguration>& configuration) const;

        //! \copydoc EnvironmentBase
        [[nodiscard]] float longestPath() const final override;
    };

    void from_json(const nlohmann::json& j, PointGraphEnvironment& environment);

}  // namespace grstapse