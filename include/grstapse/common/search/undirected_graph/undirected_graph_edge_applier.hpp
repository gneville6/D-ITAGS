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
#include "grstapse/common/search/edge_applier_base.hpp"
#include "grstapse/common/search/undirected_graph/undirected_graph.hpp"

namespace grstapse
{
    /**!
     * Applies an edge from an undirected graph
     *
     * \tparam UndirectedGraphSearchNodeDeriv A derivative of UndirectedGraphSearchNodeBase
     */
    template <typename UndirectedGraphSearchNodeDeriv>
    class UndirectedGraphEdgeApplier : public EdgeApplierBase<UndirectedGraphSearchNodeDeriv>
    {
        using Edge = typename UndirectedGraphSearchNodeDeriv::Edge;

       public:
        //! Constructor
        explicit UndirectedGraphEdgeApplier(const std::shared_ptr<Edge>& edge)
            : m_edge(edge)
        {}

        //! \copydoc EdgeApplierBase
        bool isApplicable(const std::shared_ptr<const UndirectedGraphSearchNodeDeriv>& base) const final override
        {
            return m_edge->contains(base->vertex());
        }

        //! \copydoc EdgeApplierBase
        std::shared_ptr<UndirectedGraphSearchNodeDeriv> apply(
            const std::shared_ptr<const UndirectedGraphSearchNodeDeriv>& base) const final override
        {
            const std::shared_ptr<typename UndirectedGraphSearchNodeDeriv::Vertex>& other =
                m_edge->other(base->vertex());
            return std::make_shared<UndirectedGraphSearchNodeDeriv>(other, m_edge, base);
        }

       protected:
        std::shared_ptr<Edge> m_edge;
    };

}  // namespace grstapse