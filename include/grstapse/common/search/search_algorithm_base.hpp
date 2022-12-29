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
#include "grstapse/common/search/search_parameters.hpp"
#include "grstapse/common/search/search_results.hpp"
#include "grstapse/common/utilities/noncopyable.hpp"
#include "grstapse/common/utilities/timer.hpp"
#include "grstapse/common/utilities/timer_runner.hpp"

namespace grstapse {
    /**!
     * \brief Abstract base class for a graph/tree search algorithm
     *
     * \tparam SearchNode A derivative of SearchNodeBase
     * \tparam SearchStatistics A derivative of SearchStatisticsBase
     */
    template<SearchNodeDeriv SearchNode, SearchStatisticsDeriv SearchStatistics = SearchStatisticsCommon>
    class SearchAlgorithmBase : public Noncopyable {
    public:
        //! \returns The root node for search
        [[nodiscard]] virtual std::shared_ptr<SearchNode> createRootNode() = 0;

        /**!
         * Conducts a search starting at \p root
         *
         * \returns The results of the search
         */
        virtual SearchResults<SearchNode, SearchStatistics> searchFromNode(const std::shared_ptr<SearchNode> &node) = 0;

        std::shared_ptr<const SearchParameters> m_parameters;
    public:
        /**!
         * Conducts a search
         *
         * Creates a root node using SearchAlgorithmBase::createRootNode and then starts
         * a search
         *
         * \returns The results of the search
         */
        virtual SearchResults<SearchNode, SearchStatistics> search() {
            TimerRunner timer_runner(m_parameters->timer_name);
            m_root = createRootNode();
            return searchFromNode(m_root);
        }

    protected:
        //! \brief Default Constructor
        explicit SearchAlgorithmBase(const std::shared_ptr<const SearchParameters> &parameters)
                : m_statistics(std::make_shared<SearchStatistics>()), m_parameters(parameters) {}

        std::shared_ptr<SearchStatistics> m_statistics;
        std::shared_ptr<SearchNode> m_root;
    };
}  // namespace grstapse