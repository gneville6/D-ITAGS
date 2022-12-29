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
#include "grstapse/common/search/search_node_base.hpp"
#include "grstapse/common/search/search_statistics_common.hpp"

namespace grstapse
{
    /**!
     * \brief Container for the results of a search
     *
     * \tparam SearchNode A derivative of SearchNodeBase
     * \tparam SearchStatistics A derivative of SearchStatisticsBase
     */
    template <SearchNodeDeriv SearchNode, SearchStatisticsDeriv SearchStatistics = SearchStatisticsCommon>
    class SearchResults
    {
       public:
        /**!
         * Constructor
         *
         * \param goal The final node in the search
         * \param statistics The statistics from the search
         */
        explicit SearchResults(const std::shared_ptr<SearchNode>& goal,
                               const std::shared_ptr<SearchStatistics>& statistics)
            : m_goal(goal)
            , m_statistics(statistics)
        {}

        //! \returns Whether the goal was found during search
        [[nodiscard]] bool foundGoal() const
        {
            return m_goal != nullptr;
        }

        //! \returns The goal found during a search
        [[nodiscard]] const std::shared_ptr<SearchNode>& goal()
        {
            return m_goal;
        }

        //! \returns The goal found during a search
        [[nodiscard]] std::shared_ptr<const SearchNode> goal() const
        {
            return m_goal;
        }

        //! \returns The statistics of the search that produced this result
        [[nodiscard]] const std::shared_ptr<SearchStatistics>& statistics()
        {
            return m_statistics;
        }

        //! \returns The statistics of the search that produced this result
        [[nodiscard]] std::shared_ptr<const SearchStatistics> statistics() const
        {
            return m_statistics;
        }

       private:
        std::shared_ptr<SearchNode> m_goal;
        std::shared_ptr<SearchStatistics> m_statistics;
    };
}  // namespace grstapse