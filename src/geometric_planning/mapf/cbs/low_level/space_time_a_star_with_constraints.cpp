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
#include "grstapse/geometric_planning/mapf/cbs/low_level/space_time_a_star_with_constraints.hpp"

// Local
#include "grstapse/common/search/null_memoization.hpp"
#include "grstapse/geometric_planning/grid/grid_cell_manhattan_distance.hpp"
#include "grstapse/geometric_planning/mapf/cbs/low_level/grid_cell_cardinals_plus_wait_generator.hpp"
#include "grstapse/geometric_planning/mapf/cbs/low_level/prune_constraints.hpp"
#include "grstapse/geometric_planning/mapf/cbs/low_level/space_time_a_star_with_constraints_parameters.hpp"
#include "grstapse/geometric_planning/mapf/cbs/low_level/temporal_grid_cell_goal_check_with_constraints.hpp"
#include "grstapse/geometric_planning/mapf/cbs/low_level/temporal_grid_cell_path_cost.hpp"

namespace grstapse
{
    SpaceTimeAStarWithConstraints::SpaceTimeAStarWithConstraints(
        const std::shared_ptr<const SpaceTimeAStarParameters>& parameters,
        const std::shared_ptr<const GridMap>& map,
        const std::shared_ptr<const GridCell>& initial,
        const std::shared_ptr<const GridCell>& goal,
        const robin_hood::unordered_set<std::shared_ptr<const ConstraintBase>>& constraints)
        : Base{.parameters = parameters,
               .functors   = {.path_cost = std::make_shared<const TemporalGridCellPathCost>(),
                              .heuristic = std::make_shared<const GridCellManhattanDistance<TemporalGridCellNode>>(goal),
                              .successor_generator = std::make_shared<const GridCellCardinalsPlusWaitGenerator>(map),
                              .goal_check =
                                std::make_shared<const TemporalGridCellGoalCheckWithConstraints>(goal, constraints),
                              .memoization    = std::make_shared<const NullMemoization<TemporalGridCellNode>>(),
                              .pruning_method = std::make_shared<const PruneConstraints>(constraints)}}
        , m_initial(initial)
    {}

    std::shared_ptr<TemporalGridCellNode> SpaceTimeAStarWithConstraints::createRootNode()
    {
        auto root = std::make_shared<TemporalGridCellNode>(0, m_initial->x(), m_initial->y(), nullptr);
        root->setG(0);
        root->setH(0);
        return root;
    }
}  // namespace grstapse