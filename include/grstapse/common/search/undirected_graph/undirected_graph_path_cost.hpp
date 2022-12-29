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
#include "grstapse/common/search/path_cost_base.hpp"
#include "grstapse/common/search/undirected_graph/undirected_graph_search_node.hpp"

namespace grstapse
{
    /**!
     * Computes the path cost for a search in an undirected graph
     *
     * \tparam UndirectedGraphSearchNodeDeriv A derivative of UndirectedGraphSearchNodeBase
     */
    template <typename UndirectedGraphSearchNodeDeriv>
    class UndirectedGraphPathCost : public PathCostBase<UndirectedGraphSearchNodeDeriv>
    {
       public:
        //! \copydoc PathCostBase
        [[nodiscard]] virtual float operator()(const std::shared_ptr<UndirectedGraphSearchNodeDeriv>& node) const
        {
            return node->g() + node->lastEdge()->cost();
        }
    };
}  // namespace grstapse