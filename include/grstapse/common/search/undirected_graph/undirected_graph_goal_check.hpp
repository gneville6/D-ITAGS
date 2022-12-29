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
#include "grstapse/common/search/goal_check_base.hpp"
#include "grstapse/common/search/undirected_graph/undirected_graph.hpp"

namespace grstapse
{
    /**!
     * Checks if a search node reaches the specified vertex
     *
     * \tparam UndirectedGraphSearchNodeDeriv A derivative of UndirectedGraphSearchNodeBase
     */
    template <typename UndirectedGraphSearchNodeDeriv>
    class UndirectedGraphGoalCheck : public GoalCheckBase<UndirectedGraphSearchNodeDeriv>
    {
        using Vertex = typename UndirectedGraphSearchNodeDeriv::Vertex;

       public:
        UndirectedGraphGoalCheck(const std::shared_ptr<Vertex>& goal)
            : m_goal(goal)
        {}

        //! \copydoc GoalCheckBase
        [[nodiscard]] virtual bool operator()(
            const std::shared_ptr<const UndirectedGraphSearchNodeDeriv>& node) const override
        {
            return node->vertex() == m_goal;
        }

       protected:
        std::shared_ptr<Vertex> m_goal;
    };
}  // namespace grstapse