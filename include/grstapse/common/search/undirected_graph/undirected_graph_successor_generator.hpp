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
#include "grstapse/common/search/successor_generator_base.hpp"
#include "grstapse/common/search/undirected_graph/undirected_graph.hpp"
#include "grstapse/common/search/undirected_graph/undirected_graph_edge_applier.hpp"

namespace grstapse
{
    /**!
     * Generates the successors for a node based upon a finite undirected graph
     *
     * \tparam UndirectedGraphSearchNodeDeriv A derivative of UndirectedGraphSearchNodeBase
     */
    template <typename UndirectedGraphSearchNodeDeriv>
    class UndirectedGraphSuccessorGenerator : public SuccessorGeneratorBase<UndirectedGraphSearchNodeDeriv>
    {
        using Base = SuccessorGeneratorBase<UndirectedGraphSearchNodeDeriv>;

       public:
        //! Constructor
        explicit UndirectedGraphSuccessorGenerator(
            const std::shared_ptr<typename UndirectedGraphSearchNodeDeriv::Graph>& graph)
        {
            for(const auto& [key, edge]: graph->edges())
            {
                Base::m_edge_appliers.push_back(
                    std::make_shared<UndirectedGraphEdgeApplier<UndirectedGraphSearchNodeDeriv>>(edge));
            }
        }

       protected:
        //! \copydoc SuccessorGeneratorBase
        bool isValidNode(const std::shared_ptr<const UndirectedGraphSearchNodeDeriv>& node) const final override
        {
            return true;
        }
    };

}  // namespace grstapse