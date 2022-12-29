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
#include "grstapse/common/search/best_first_search_node_base.hpp"
#include "grstapse/common/search/goal_check_base.hpp"
#include "grstapse/common/search/heuristic_base.hpp"
#include "grstapse/common/search/memoization_base.hpp"
#include "grstapse/common/search/null_memoization.hpp"
#include "grstapse/common/search/null_pruning_method.hpp"
#include "grstapse/common/search/pruning_method_base.hpp"
#include "grstapse/common/search/successor_generator_base.hpp"

namespace grstapse
{
    /**!
     * Container to hold the various functors for best first search
     *
     * \tparam SearchNode A derivative of SearchNodeBase
     */
    template <SearchNodeDeriv SearchNode>
    struct BestFirstSearchFunctors
    {
        using Heuristic          = HeuristicBase<SearchNode>;
        using SuccessorGenerator = SuccessorGeneratorBase<SearchNode>;
        using GoalCheck          = GoalCheckBase<SearchNode>;
        using Memoization        = MemoizationBase<SearchNode>;
        using PruningMethod      = PruningMethodBase<SearchNode>;

        BestFirstSearchFunctors(const std::shared_ptr<const Heuristic>& heuristic,
                                const std::shared_ptr<const SuccessorGenerator>& successor_generator,
                                const std::shared_ptr<const GoalCheck>& goal_check,
                                const std::shared_ptr<const Memoization>& memoization =
                                    std::make_shared<const NullMemoization<SearchNode>>(),
                                const std::shared_ptr<const PruningMethod>& prepruning_method =
                                    std::make_shared<const NullPruningMethod<SearchNode>>(),
                                const std::shared_ptr<const PruningMethod>& postpruning_method =
                                    std::make_shared<const NullPruningMethod<SearchNode>>())
            : heuristic(heuristic)
            , successor_generator(successor_generator)
            , goal_check(goal_check)
            , memoization(memoization)
            , prepruning_method(prepruning_method)
            , postpruning_method(postpruning_method)
        {}

        std::shared_ptr<const Heuristic> heuristic;
        std::shared_ptr<const SuccessorGenerator> successor_generator;
        std::shared_ptr<const GoalCheck> goal_check;
        std::shared_ptr<const Memoization> memoization;
        std::shared_ptr<const PruningMethod> prepruning_method;
        std::shared_ptr<const PruningMethod> postpruning_method;
    };

}  // namespace grstapse