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
#include "grstapse/common/search/heuristic_base.hpp"
#include "grstapse/common/search/undirected_graph/undirected_graph_search_node.hpp"
#include "grstapse/geometric_planning/graph/point/point_graph_configuration.hpp"

namespace grstapse
{
    /**!
     * A heuristic that uses euclidean distance to the goal
     *
     * \tparam UndirectedGraphSearchNodeDeriv
     *
     * \todo Check that PointGraphConfiguration is the VertexPayload
     */
    template <typename UndirectedGraphSearchNodeDeriv>
    class PointGraphConfigurationEuclideanDistanceHeuristic : public HeuristicBase<UndirectedGraphSearchNodeDeriv>
    {
       public:
        //! Constructor
        explicit PointGraphConfigurationEuclideanDistanceHeuristic(
            const std::shared_ptr<const PointGraphConfiguration>& goal)
            : m_goal(goal)
        {}

        //! \copydoc HeuristicBase
        [[nodiscard]] virtual float operator()(
            const std::shared_ptr<UndirectedGraphSearchNodeDeriv>& node) const final override
        {
            return node->vertex()->payload()->euclideanDistance(m_goal);
        }

       private:
        std::shared_ptr<const PointGraphConfiguration> m_goal;
    };

}  // namespace grstapse