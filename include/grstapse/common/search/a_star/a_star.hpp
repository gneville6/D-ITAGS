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

// Local
#include "grstapse/common/search/a_star/a_star_functors.hpp"
#include "grstapse/common/search/a_star/a_star_search_node_base.hpp"
#include "grstapse/common/search/best_first_search_base.hpp"
#include "grstapse/common/search/path_cost_base.hpp"

namespace grstapse
{
    /**
     * Conducts a generic A* search to find the shortest path if given an admissable heuristic
     *
     * A* is an informed search algorithm that find the shortest path from an initial node to a
     * goal node if given an admissible heuristic
     *
     * \tparam SearchNode A derivative of AStarSearchNodeBase
     * \tparam SearchStatistics A derivative of
     */
    template <AStarSearchNodeDeriv SearchNode, SearchStatisticsDeriv SearchStatistics = SearchStatisticsCommon>
    class AStar : public BestFirstSearchBase<SearchNode, SearchStatistics>
    {
        using Base     = BestFirstSearchBase<SearchNode, SearchStatistics>;
        using PathCost = PathCostBase<SearchNode>;

       public:
        /**!
         * Constructor
         *
         * \param parameters
         * \param functors
         */
        explicit AStar(const std::shared_ptr<const BestFirstSearchParameters>& parameters,
                       const AStarFunctors<SearchNode>& functors)
            : Base{.parameters = parameters, .functors = functors}
            , m_path_cost(functors.path_cost)
        {}

       protected:
        //! Compute the path cost and heuristic value of a node
        void evaluateNode(const std::shared_ptr<SearchNode>& node) final override
        {
            // Path Cost
            {
                TimerRunner timer_runner(Base::m_parameters->timer_name + "_pathcost");
                node->setG(m_path_cost->operator()(node));
            }

            // Heuristic
            {
                TimerRunner timer_runner(Base::m_parameters->timer_name + "_heuristic");
                node->setH(Base::m_heuristic->operator()(node));
            }
        }

        std::shared_ptr<const PathCost> m_path_cost;
    };
}  // namespace grstapse