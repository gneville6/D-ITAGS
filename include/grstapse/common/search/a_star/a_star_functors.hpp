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
#include <concepts>
#include <memory>

// Local
#include "grstapse/common/search/a_star/a_star_search_node_base.hpp"
#include "grstapse/common/search/best_first_search_functors.hpp"
#include "grstapse/common/search/path_cost_base.hpp"

namespace grstapse
{
    /**!
     * A container for functors used by A*
     *
     * \tparam SearchNode A derivative of AStarSearchNodeBase
     */
    template <AStarSearchNodeDeriv SearchNode>
    class AStarFunctors : public BestFirstSearchFunctors<SearchNode>
    {
        using Base = BestFirstSearchFunctors<SearchNode>;

       public:
        using PathCost = PathCostBase<SearchNode>;

        /**!
         * Constructor
         *
         * \param path_cost
         * \param heuristic
         * \param successor_generator
         * \param goal_check
         * \param memoization
         * \param prepruning_method
         * \param postpruning_method
         */
        AStarFunctors(const std::shared_ptr<const PathCost>& path_cost,
                      const std::shared_ptr<const typename Base::Heuristic>& heuristic,
                      const std::shared_ptr<const typename Base::SuccessorGenerator>& successor_generator,
                      const std::shared_ptr<const typename Base::GoalCheck>& goal_check,
                      const std::shared_ptr<const typename Base::Memoization>& memoization =
                          std::make_shared<const NullMemoization<SearchNode>>(),
                      const std::shared_ptr<const typename Base::PruningMethod>& prepruning_method =
                          std::make_shared<const NullPruningMethod<SearchNode>>(),
                      const std::shared_ptr<const typename Base::PruningMethod>& postpruning_method =
                          std::make_shared<const NullPruningMethod<SearchNode>>())
            : Base{.heuristic           = heuristic,
                   .successor_generator = successor_generator,
                   .goal_check          = goal_check,
                   .memoization         = memoization,
                   .prepruning_method   = prepruning_method,
                   .postpruning_method  = postpruning_method}
            , path_cost(path_cost)
        {}

        std::shared_ptr<const PathCost> path_cost;
    };

}  // namespace grstapse