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
#include "grstapse/geometric_planning/mapf/cbs/conflict_based_search_parameters.hpp"

namespace grstapse
{
    ConflictBasedSearchParameters::ConflictBasedSearchParameters(
        ConstraintTreeNodeCostType cost_type,
        const std::shared_ptr<const GridMap>& map,
        const std::vector<std::shared_ptr<const GridCell>>& initial_states,
        const std::vector<std::shared_ptr<const GridCell>>& goal_states,
        const std::string& high_level_timer_name,
        const std::string& low_level_timer_name,
        bool has_timeout,
        float timeout)
        : MultiAgentPathFindingParameters{.map = map, .initial_states = initial_states, .goal_states = goal_states}
        , SearchParameters{.has_timeout = has_timeout, .timeout = timeout, .timer_name = high_level_timer_name}
        , cost_type(cost_type)
        , low_level_timer_name(low_level_timer_name)
    {}
}  // namespace grstapse