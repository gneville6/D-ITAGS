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
#include "grstapse/common/search/best_first_search_base.hpp"
#include "grstapse/common/search/focal_a_star/focal_a_star_functors.hpp"
#include "grstapse/common/search/focal_a_star/focal_a_star_parameters.hpp"
#include "grstapse/common/search/focal_a_star/focal_a_star_search_node_base.hpp"
#include "grstapse/common/search/focal_a_star/focal_wrapper.hpp"

namespace grstapse
{
    /**
     * Search algorithm to find the shortest path
     * within a given suboptimality bound (also known as focal search)
     *
     * \tparam SearchNodeDeriv A derivative of FocalAStarSearchNodeBase
     *
     * \cite "Studies in Semi-Admissible Heuristics." IEEE Trans. Pattern Anal. Mach. Intell.
     *        4(4): 392-399 (1982)
     */
    template <typename SearchNodeDeriv>
    requires std::derived_from<SearchNodeDeriv, FocalAStarSearchNodeBase<SearchNodeDeriv>>
    class FocalAStar : public BestFirstSearchBase<SearchNodeDeriv>
    {
        using Base           = BestFirstSearchBase<SearchNodeDeriv>;
        using PathCost       = PathCostBase<SearchNodeDeriv>;
        using FocalHeuristic = FocalHeuristicBase<SearchNodeDeriv>;

       public:
        /**!
         * Constructor
         *
         * \param parameters
         * \param heuristic A derivative of HeuristicBase
         * \param path_cost A derivative of PathCostBase
         * \param focal_heuristic A derivative of FocalHeuristicBase
         * \param node_expander A derivative of SuccessorGenerator
         * \param goal_check A derivative of GoalCheckBase
         * \param memoization A derivative of MemoizationBase
         * \param pruning_method A derivative of PruningMethodBase
         */
        FocalAStar(std::shared_ptr<const FocalAStarParameters> parameters,
                   const FocalAStarFunctors<SearchNodeDeriv>& functors)
            : Base(parameters, functors)
            , m_path_cost(functors.path_cost)
            , m_focal_heuristic(functors.focal_heuristic)
        {}

        SearchResults<SearchNodeDeriv> searchFromNode(std::shared_ptr<SearchNodeDeriv> root) override
        {
            assert(root);
            Base::m_timer.start();
            Base::m_statistics->incrementNodesGenerated();

            MutablePriorityQueue<unsigned int, float, FocalWrapper<SearchNodeDeriv>> focal;

            Base::m_open.push(Base::m_memoization->operator()(root), root);
            Base::m_focal.push(Base::m_memoization->operator()(root), root);
            float best_f = root->f();

            const float w          = std::dynamic_pointer_cast<FocalAStarParameters>(Base::m_parameters)->w();
            const bool rebuild     = std::dynamic_pointer_cast<FocalAStarParameters>(Base::m_parameters)->rebuild();
            const bool has_timeout = Base::m_parameters->hasTimeout();
            const float timeout    = Base::m_parameters->timeout();
            const bool pruning_before_eval = Base::m_parameters->pruneBeforeEval();

            // Continue through open set until it is empty or timeout
            // Only check timeout if parameter is set
            while(!focal.empty() && (!has_timeout || Base::m_timer.get() < timeout))
            {
                std::shared_ptr<SearchNodeDeriv> base = focal.pop()->internal();
                Base::m_open.erase(Base::m_memoization->operator()(base));

                if(Base::m_goal_check->operator()(base))
                {
                    Base::m_timer.stop();
                    Base::m_statistics->setTotalTime(Base::m_timer.get());
                    return SearchResults<SearchNodeDeriv>(base, Base::m_statistics);
                }
                // TODO(Andrew): Should this be before the goal check? (Only matters for anytime/repair)
                Base::m_closed.push_back(base);
                Base::m_closed_ids.insert(Base::m_memoization->operator()(base));
                base->setStatus(SearchNodeStatus::e_closed);

                // Expand
                std::vector<std::shared_ptr<SearchNodeDeriv>> children = Base::m_successor_generator->operator()(base);
                Base::m_statistics->incrementNodesExpanded();

                if(children.empty())
                {
                    base->setStatus(SearchNodeStatus::e_deadend);
                    Base::m_statistics->incrementNodesDeadend();
                }
                else
                {
                    Base::m_statistics->incrementNodesGenerated(children.size());
                }

                for(std::shared_ptr<SearchNodeDeriv> child: children)
                {
                    const unsigned int id = Base::m_memoization->operator()(child);

                    // Ignore if this node has already been expanded
                    if(Base::m_closed_ids.find(id) != Base::m_closed_ids.end())
                    {
                        continue;
                    }

                    // Check if the child should be pruned before evaluation
                    if(pruning_before_eval && Base::m_pruning_method->operator()(child))
                    {
                        child->setStatus(SearchNodeStatus::e_pruned);
                        Base::m_statistics->incrementNodesPruned();
                        Base::m_pruned.push_back(child);
                        continue;
                    }

                    // Evaluate
                    evaluateNode(child);
                    Base::m_statistics->incrementNodesEvaluated();

                    // Check if child should be pruned
                    if(!pruning_before_eval && Base::m_pruning_method->operator()(child))
                    {
                        child->setStatus(SearchNodeStatus::e_pruned);
                        Base::m_statistics->incrementNodesPruned();
                        Base::m_pruned.push_back(child);
                        continue;
                    }

                    // Add child to open set
                    child->setStatus(SearchNodeStatus::e_open);
                    Base::m_open.push(id, child);
                }

                if(rebuild)
                {
                    focal.clear();
                    best_f = Base::m_open.top()->f();
                    for(auto it = Base::m_open.ordered_begin(), end = Base::m_open.ordered_end(); it != end; ++it)
                    {
                        std::shared_ptr<SearchNodeDeriv> node = *it;
                        float value                           = node->f();
                        if(value <= best_f * w)
                        {
                            focal.push(Base::m_memoization->operator()(node),
                                       std::make_shared<FocalWrapper<SearchNodeDeriv>>(node));
                        }
                        else
                        {
                            break;
                        }
                    }
                }
                else
                {
                    float previous_best_f = best_f;
                    best_f                = Base::m_open.top()->f();
                    if(best_f > previous_best_f)
                    {
                        for(auto it = Base::m_open.ordered_begin(), end = Base::m_open.ordered_end(); it != end; ++it)
                        {
                            std::shared_ptr<SearchNodeDeriv> node = *it;
                            float value                           = node->f();
                            if(value > previous_best_f * w && value <= best_f * w)
                            {
                                focal.push(Base::m_memoization->operator()(node),
                                           std::make_shared<FocalWrapper<SearchNodeDeriv>>(node));
                            }
                            if(value > best_f * w)
                            {
                                break;
                            }
                        }
                    }
                }
            }
            Base::m_timer.stop();
            Base::m_statistics->setTotalTime(Base::m_timer.get());
            return SearchResults<SearchNodeDeriv>(nullptr, Base::m_statistics);
        }

       protected:
        //! Compute the path cost and heuristic value of a node
        virtual void evaluateNode(std::shared_ptr<SearchNodeDeriv> node) override
        {
            node->setG(m_path_cost->operator()(node));
            node->setH(Base::m_heuristic->operator()(node));
            node->setFocalH(m_focal_heuristic->operator()(node));
        }

        std::unique_ptr<const PathCost> m_path_cost;
        std::unique_ptr<const FocalHeuristic> m_focal_heuristic;
    };
}  // namespace grstapse