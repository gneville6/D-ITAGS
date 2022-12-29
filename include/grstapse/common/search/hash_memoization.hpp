
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
#include "grstapse/common/search/memoization_base.hpp"

namespace grstapse
{
    /**!
     * A memoization method that just uses the node's SearchNodeBase::hash function
     *
     * \tparam SearchNodeDeriv A derivative of SearchNodeBase
     */
    template <SearchNodeDeriv SearchNode>
    class HashMemoization : public MemoizationBase<SearchNode>
    {
       public:
        //! Default Constructor
        HashMemoization() = default;

        //! \returns The node's unique identifier
        [[nodiscard]] inline unsigned int operator()(const std::shared_ptr<const SearchNode>& node) const final
        {
            return node->hash();
        }
    };
}  // namespace grstapse