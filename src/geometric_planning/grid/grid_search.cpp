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
#include "grstapse/geometric_planning/grid/grid_search.hpp"

// Local
#include "grstapse/geometric_planning/grid/grid_cell_cardinals_generator.hpp"
#include "grstapse/geometric_planning/grid/grid_cell_euclidean_distance.hpp"
#include "grstapse/geometric_planning/grid/grid_cell_goal_check.hpp"
#include "grstapse/geometric_planning/grid/grid_cell_path_cost.hpp"

namespace grstapse
{
    GridSearch::GridSearch(const std::shared_ptr<const BestFirstSearchParameters>& parameters,
                           const std::shared_ptr<const GridMap>& map,
                           const std::shared_ptr<const GridCell>& initial,
                           const std::shared_ptr<const GridCell>& goal)
        : Base{.parameters = parameters,
               .functors   = {.path_cost = std::make_shared<const GridCellPathCost<GridCellNode>>(),
                              .heuristic = std::make_shared<const GridCellEuclideanDistance<GridCellNode>>(goal),
                              .successor_generator = std::make_shared<const GridCellCardinalsGenerator>(map),
                              .goal_check          = std::make_shared<const GridCellGoalCheck<GridCellNode>>(goal)}}
        , m_initial(initial)
    {}

    std::shared_ptr<GridCellNode> GridSearch::createRootNode()
    {
        auto root = std::make_shared<GridCellNode>(m_initial->x(), m_initial->y(), nullptr);
        root->setG(0.0);
        root->setH(0.0);
        return root;
    }
}  // namespace grstapse