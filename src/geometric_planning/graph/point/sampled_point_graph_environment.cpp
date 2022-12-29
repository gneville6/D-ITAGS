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
#include "grstapse/geometric_planning/graph/point/sampled_point_graph_environment.hpp"

// Local
#include "grstapse/common/utilities/constants.hpp"
#include "grstapse/geometric_planning/graph/point/point_graph_environment.hpp"

namespace grstapse
{
    SampledPointGraphEnvironment::SampledPointGraphEnvironment()
        : GraphEnvironment(GraphType::e_sampled_point)
    {}

    const std::shared_ptr<PointGraphEnvironment>& SampledPointGraphEnvironment::graph(unsigned int index) const
    {
        assert(index < m_graphs.size());
        return m_graphs[index];
    }

    float SampledPointGraphEnvironment::longestPath() const
    {
        // TODO(andrew)
        throw std::logic_error("Not implemented");
    }

    void from_json(const nlohmann::json& j, SampledPointGraphEnvironment& environment)
    {
        auto pge_base = std::make_shared<PointGraphEnvironment>();
        for(const nlohmann::json& vertex_j: j[constants::k_vertices])
        {
            pge_base->addVertex(vertex_j[constants::k_id],
                                std::make_shared<PointGraphConfiguration>(vertex_j[constants::k_id],
                                                                          vertex_j[constants::k_x],
                                                                          vertex_j[constants::k_y]));
        }

        for(const nlohmann::json& edges_j: j[constants::k_edges])
        {
            auto pge = pge_base->shallowCopy();
            for(const nlohmann::json& edge_j: edges_j)
            {
                pge->addEdge(edge_j[constants::k_vertex_a], edge_j[constants::k_vertex_b], edge_j[constants::k_cost]);
            }
            environment.addGraph(pge);
        }
    }
}  // namespace grstapse