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
#include <cassert>
#include <memory>
#include <set>
#include <vector>

// Local
#include "grstapse/common/search/best_first_search_functors.hpp"
#include "grstapse/common/search/best_first_search_node_base.hpp"
#include "grstapse/common/search/best_first_search_parameters.hpp"
#include "grstapse/common/search/search_algorithm_base.hpp"
#include "grstapse/common/utilities/mutable_priority_queue/mutable_priority_queue.hpp"
#include "grstapse/common/utilities/time_keeper.hpp"

namespace grstapse {

    /**
     * Abstract Base class for all best first searches (A*, Greedy Best First Search, etc)
     *
     * \tparam SearchNode A derivative of BestFirstSearchNodeBase
     * \tparam SearchStatistics A derivative of SearchStatisticsBase
     *
     * \todo Add continue search function for anytime
     */
    template<BestFirstSearchNodeDeriv SearchNode, SearchStatisticsDeriv SearchStatistics = SearchStatisticsCommon>
    class BestFirstSearchBase : public SearchAlgorithmBase<SearchNode, SearchStatistics> {
        using Base = SearchAlgorithmBase<SearchNode, SearchStatistics>;

    protected:
        using Heuristic = HeuristicBase<SearchNode>;
        using SuccessorGenerator = SuccessorGeneratorBase<SearchNode>;
        using GoalCheck = GoalCheckBase<SearchNode>;
        using Memoization = MemoizationBase<SearchNode>;
        using PruningMethod = PruningMethodBase<SearchNode>;

    public:
        /**!
         * Constructor
         *
         * \param parameters The parameters for a best first search
         * \param functors A container for the various functors used by best first search (heuristic, goal check, etc)
         */
        BestFirstSearchBase(const std::shared_ptr<const BestFirstSearchParameters> &parameters,
                            const BestFirstSearchFunctors<SearchNode> &functors)
                : Base(parameters), m_heuristic(functors.heuristic),
                  m_successor_generator(functors.successor_generator), m_goal_check(functors.goal_check),
                  m_memoization(functors.memoization), m_prepruning_method(functors.prepruning_method),
                  m_postpruning_method(functors.postpruning_method) {}

        /**
         * Runs the search
         *
         * \returns The results of the search (solution and statistics)
         */
        SearchResults<SearchNode, SearchStatistics> searchFromNode(const std::shared_ptr<SearchNode> &root) override {
            assert(root);
            Base::m_statistics->incrementNodesGenerated();
            m_open.push(m_memoization->operator()(root), root);

            auto bfs_parameters = std::dynamic_pointer_cast<const BestFirstSearchParameters>(Base::m_parameters);
            const bool has_prepruning = m_prepruning_method != nullptr;
            const bool has_postpruning = m_postpruning_method != nullptr;

            // Continue through open set until it is empty or timeout
            // Only check timeout if parameter is set
            while (!m_open.empty() &&
                   (!bfs_parameters->has_timeout ||
                    TimeKeeper::instance().time(bfs_parameters->timer_name) < bfs_parameters->timeout)) {
                std::shared_ptr<SearchNode> base = m_open.pop();

                // Close node before the goal check for future anytime/repair
                if (bfs_parameters->save_closed_nodes) {
                    m_closed.push_back(base);
                }
                m_closed_ids.insert(m_memoization->operator()(base));
                base->setStatus(SearchNodeStatus::e_closed);

                // Check if goal node
                if (m_goal_check->operator()(base)) {
                    return SearchResults<SearchNode, SearchStatistics>(base, Base::m_statistics);
                }

                // Generate successors
                std::vector<std::shared_ptr<SearchNode>> children = m_successor_generator->operator()(base);
                Base::m_statistics->incrementNodesExpanded();

                if (children.empty()) {
                    base->setStatus(SearchNodeStatus::e_deadend);
                    Base::m_statistics->incrementNodesDeadend();
                } else {
                    Base::m_statistics->incrementNodesGenerated(children.size());
                }

                for (std::shared_ptr<SearchNode> child: children) {
                    const unsigned int id = m_memoization->operator()(child);

                    // Ignore if this node has already been closed or pruned
                    if (m_closed_ids.find(id) != m_closed_ids.end() || m_pruned_ids.find(id) != m_pruned_ids.end()) {
                        continue;
                    }

                    // Check if the child should be pruned before evaluation
                    if (has_prepruning && m_prepruning_method->operator()(child)) {
                        child->setStatus(SearchNodeStatus::e_pruned);
                        Base::m_statistics->incrementNodesPruned();
                        m_pruned_ids.insert(id);
                        if (bfs_parameters->save_pruned_nodes) {
                            m_pruned.push_back(child);
                        }
                        continue;
                    }

                    // Evaluate
                    evaluateNode(child);
                    Base::m_statistics->incrementNodesEvaluated();

                    // Check if child should be pruned after evaluation
                    if (has_postpruning && m_postpruning_method->operator()(child)) {
                        child->setStatus(SearchNodeStatus::e_pruned);
                        Base::m_statistics->incrementNodesPruned();
                        m_pruned_ids.insert(id);
                        if (bfs_parameters->save_pruned_nodes) {
                            m_pruned.push_back(child);
                        }
                        continue;
                    }

                    // Add child to open set
                    child->setStatus(SearchNodeStatus::e_open);
                    m_open.push(id, child);
                }
            }
            return SearchResults<SearchNode, SearchStatistics>(nullptr, Base::m_statistics);
        }

        /**
         * Continues a previously started search starting with the top of the open set
         *
         * \returns a search results object with the results from the continued search
         */
        SearchResults<SearchNode, SearchStatistics> continueSearch() {
            TimerRunner timer_runner(Base::m_parameters->timer_name);
            if (!m_open.empty()) {
                std::shared_ptr<SearchNode> startNode = m_open.pop();
                return searchFromNode(startNode);
            } else {
                return SearchResults<SearchNode>(nullptr, Base::m_statistics);
            }
        }

    protected:
        /**!
         * Evaluate the value of a node
         *
         * \param node The node to evaluate
         */
        virtual void evaluateNode(const std::shared_ptr<SearchNode> &node) = 0;

        std::shared_ptr<const Heuristic> m_heuristic;
        std::shared_ptr<const SuccessorGenerator> m_successor_generator;
        std::shared_ptr<const GoalCheck> m_goal_check;
        std::shared_ptr<const Memoization> m_memoization;
        std::shared_ptr<const PruningMethod> m_prepruning_method;
        std::shared_ptr<const PruningMethod> m_postpruning_method;

        MutablePriorityQueue<unsigned int, float, SearchNode> m_open;  //!< key, priority, payload

        std::vector<std::shared_ptr<SearchNode>> m_closed;
        std::set<unsigned int> m_closed_ids;

        std::vector<std::shared_ptr<SearchNode>> m_pruned;
        std::set<unsigned int> m_pruned_ids;
    };
}  // namespace grstapse