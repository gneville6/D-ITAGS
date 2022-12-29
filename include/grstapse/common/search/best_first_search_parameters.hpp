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
#include <limits>
// External
#include <nlohmann/json.hpp>
// Local
#include "grstapse/common/search/search_parameters.hpp"

namespace grstapse
{
    //! Parameters for a best first search
    struct BestFirstSearchParameters : public SearchParameters
    {
        BestFirstSearchParameters()
            : SearchParameters()
            , save_pruned_nodes(false)
            , save_closed_nodes(false)
        {}

        BestFirstSearchParameters(bool has_timeout,
                                  float timeout,
                                  const std::string& timer_name,
                                  bool save_pruned_nodes = false,
                                  bool save_closed_nodes = false)
            : SearchParameters{.has_timeout = has_timeout, .timeout = timeout, .timer_name = timer_name}
            , save_pruned_nodes(save_pruned_nodes)
            , save_closed_nodes(save_closed_nodes)
        {}

        bool save_pruned_nodes;
        bool save_closed_nodes;
    };
    NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(BestFirstSearchParameters,
                                       has_timeout,
                                       timeout,
                                       timer_name,
                                       save_pruned_nodes,
                                       save_closed_nodes);
}  // namespace grstapse