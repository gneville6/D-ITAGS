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

#include <type_traits>

// Local
#include "grstapse/common/search/search_node_base.hpp"
#include "grstapse/common/utilities/noncopyable.hpp"

namespace grstapse
{
    /**!
     *  An interface for defining how to check whether an edge can be applied to a node and defining how to apply the
     *  edge to the node.
     *
     * \tparam SearchNodeDeriv A derivative of SearchNodeBase
     */
    template <SearchNodeDeriv SearchNode>
    class EdgeApplierBase : private Noncopyable
    {
       public:
        //! \returns Whether this edge can be applied to \b base to create a child node
        [[nodiscard]] virtual bool isApplicable(const std::shared_ptr<const SearchNode>& base) const = 0;

        //! \returns A child node created by applying this edge
        [[nodiscard]] virtual std::shared_ptr<SearchNode> apply(
            const std::shared_ptr<const SearchNode>& base) const = 0;

       protected:
        EdgeApplierBase() = default;
    };
}  // namespace grstapse