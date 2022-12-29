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
#include "grstapse/common/search/a_star/a_star_functors.hpp"
#include "grstapse/common/search/focal_a_star/focal_a_star_search_node_base.hpp"
#include "grstapse/common/search/focal_a_star/focal_heuristic_base.hpp"

namespace grstapse
{
    template <typename SearchNodeDeriv>
    requires std::derived_from<SearchNodeDeriv, FocalAStarSearchNodeBase<SearchNodeDeriv>>
    struct FocalAStarFunctors : public AStarFunctors<SearchNodeDeriv>
    {
       private:
        using Base = AStarFunctors<SearchNodeDeriv>;

       public:
        using FocalHeuristic = FocalHeuristicBase<SearchNodeDeriv>;

        FocalAStarFunctors(std::shared_ptr<const typename Base::PathCost> path_cost,
                           std::shared_ptr<const typename Base::Heuristic> heuristic,
                           std::shared_ptr<const FocalHeuristic> focal_heuristic,
                           std::shared_ptr<const typename Base::SuccessorGenerator> successor_generator,
                           std::shared_ptr<const typename Base::GoalCheck> goal_check,
                           std::shared_ptr<const typename Base::Memoization> memoization =
                               std::make_shared<const NullMemoization<SearchNodeDeriv>>(),
                           std::shared_ptr<const typename Base::PruningMethod> pruning_method =
                               std::make_shared<const NullPruningMethod<SearchNodeDeriv>>())
            : Base{.heuristic           = heuristic,
                   .path_cost           = path_cost,
                   .successor_generator = successor_generator,
                   .goal_check          = goal_check,
                   .memoization         = memoization,
                   .pruning_method      = pruning_method}
            , focal_heuristic(focal_heuristic)
        {}

        std::shared_ptr<const FocalHeuristic> focal_heuristic;
    };

}  // namespace grstapse