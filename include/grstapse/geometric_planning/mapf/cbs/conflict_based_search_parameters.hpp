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

// Local
#include "grstapse/geometric_planning/mapf/cbs/high_level/constraint_tree_node.hpp"
#include "grstapse/geometric_planning/mapf/cbs/low_level/space_time_a_star_with_constraints_parameters.hpp"
#include "grstapse/geometric_planning/mapf/multi_agent_path_finding_parameters.hpp"

namespace grstapse
{
    /**!
     *
     */
    struct ConflictBasedSearchParameters
        : public MultiAgentPathFindingParameters
        , public SearchParameters
    {
        /**!
         *
         * \param cost_type
         * \param map
         * \param initial_states
         * \param goal_states
         */
        ConflictBasedSearchParameters(ConstraintTreeNodeCostType cost_type,
                                      const std::shared_ptr<const GridMap>& map,
                                      const std::vector<std::shared_ptr<const GridCell>>& initial_states,
                                      const std::vector<std::shared_ptr<const GridCell>>& goal_states,
                                      const std::string& high_level_timer_name,
                                      const std::string& low_level_timer_name,
                                      bool has_timeout = false,
                                      float timeout    = std::numeric_limits<float>::max());

        std::string high_level_timer_name;
        ConstraintTreeNodeCostType cost_type;
        std::string low_level_timer_name;
    };
}  // namespace grstapse