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
#include "grstapse/common/search/search_statistics_common.hpp"

// Global
#include <iostream>
// Local
#include "grstapse/common/utilities/constants.hpp"

namespace grstapse
{
    SearchStatisticsCommon::SearchStatisticsCommon()
        : m_nodes_expanded(0)
        , m_nodes_evaluated(0)
        , m_nodes_generated(0)
        , m_nodes_reopened(0)
        , m_nodes_deadend(0)
        , m_nodes_pruned(0)
    {}

    std::ostream& SearchStatisticsCommon::print(std::ostream& os) const
    {
        // TODO(Andrew): Implement
        throw std::logic_error("Not Implemented");
        return os;
    }

    void SearchStatisticsCommon::serializeToJson(nlohmann::json& j) const
    {
        j[constants::k_nodes_expanded]  = m_nodes_expanded;
        j[constants::k_nodes_evaluated] = m_nodes_evaluated;
        j[constants::k_nodes_generated] = m_nodes_generated;
        j[constants::k_nodes_reopened]  = m_nodes_reopened;
        j[constants::k_nodes_deadend]   = m_nodes_deadend;
        j[constants::k_nodes_pruned]    = m_nodes_pruned;
    }

    std::ostream& operator<<(std::ostream& os, const SearchStatisticsCommon& stats)
    {
        return stats.print(os);
    }
}  // namespace grstapse