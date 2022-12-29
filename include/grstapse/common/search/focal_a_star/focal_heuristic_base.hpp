/*
 * Graphically Recursive Simultaneous Task Allocation, Planning,
 * Scheduling, and Execution
 *
 * Copyright (C) 2020-2021
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

namespace grstapse
{
    /**!
     * Interface for computing the heuristic value of a node
     *
     * \tparam SearchNodeDeriv A derivative of FocalAStarSearchNodeBase
     */
    template <typename SearchNodeDeriv>
    requires std::derived_from<SearchNodeDeriv, FocalAStarSearchNodeBase<SearchNodeDeriv>>
    class FocalHeuristicBase : public HeuristicBase<SearchNodeDeriv>
    {
       public:
        //! Computes the focal heuristic value for a node
        [[nodiscard]] float operator()(std::shared_ptr<SearchNodeDeriv> node) const final override
        {
            return computeStateHeuristic(node) + computeTransitionHeuristic(node);
        }

       protected:
        //! Computes the focal heuristic value for a node's state
        [[nodiscard]] float computeStateHeuristic(std::shared_ptr<SearchNodeDeriv> node) const = 0;

        //! Computes the focal heuristic value for transitioning from a node's parent to the node
        [[nodiscard]] float computeTransitionHeuristic(std::shared_ptr<SearchNodeDeriv> node) const = 0;
    };

}  // namespace grstapse