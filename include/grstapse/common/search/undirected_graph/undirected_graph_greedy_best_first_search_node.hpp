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
#include "grstapse/common/search/undirected_graph/undirected_graph_search_node.hpp"

namespace grstapse
{
    /**!
     * A greedy best first search node for an undirected graph
     *
     * \tparam VertexPayload A payload type for each vertex in the undirected graph
     */
    template <typename VertexPayload = DummyPayload>
    class UndirectedGraphGreedyBestFirstSearchNode
        : public UndirectedGraphSearchNodeBase<UndirectedGraphGreedyBestFirstSearchNode,
                                               GreedyBestFirstSearchNodeBase,
                                               VertexPayload>
    {
        using Base = UndirectedGraphSearchNodeBase<UndirectedGraphGreedyBestFirstSearchNode,
                                                   GreedyBestFirstSearchNodeBase,
                                                   VertexPayload>;

       public:
        /**!
         * Constructor
         *
         * \param vertex A vertex from an undirected graph
         * \param last_edge The edge connecting the previous vertex in the search to \p vertex
         * \param parent The previous vertex in the search
         */
        UndirectedGraphGreedyBestFirstSearchNode(
            const std::shared_ptr<typename Base::Vertex>& vertex,
            const std::shared_ptr<typename Base::Edge>& last_edge                         = nullptr,
            const std::shared_ptr<const UndirectedGraphGreedyBestFirstSearchNode>& parent = nullptr)
            : Base(vertex, last_edge, parent)
        {}
    };
}  // namespace grstapse