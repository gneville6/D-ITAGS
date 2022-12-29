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
#include "grstapse/geometric_planning/mapf/cbs/low_level/grid_cell_cardinals_plus_wait_generator.hpp"

// Local
#include "grstapse/geometric_planning/grid/grid_map.hpp"
#include "grstapse/geometric_planning/mapf/cbs/low_level/temporal_grid_cell_cardinal_edge_applier.hpp"

namespace grstapse
{
    GridCellCardinalsPlusWaitGenerator::GridCellCardinalsPlusWaitGenerator(const std::shared_ptr<const GridMap>& map)
        : Base({
              std::make_shared<const TemporalGridCellCardinalEdgeApplier>(0, 1),   //  North
              std::make_shared<const TemporalGridCellCardinalEdgeApplier>(0, -1),  // South
              std::make_shared<const TemporalGridCellCardinalEdgeApplier>(1, 0),   // East
              std::make_shared<const TemporalGridCellCardinalEdgeApplier>(-1, 0),  // West
              std::make_shared<const TemporalGridCellCardinalEdgeApplier>(0, 0)    // Wait
          })
        , m_map(map)
    {}

    bool GridCellCardinalsPlusWaitGenerator::isValidNode(const std::shared_ptr<const TemporalGridCellNode>& node) const
    {
        if(node->x() >= m_map->width() || node->x() < 0)
        {
            return false;
        }

        if(node->y() >= m_map->height() || node->y() < 0)
        {
            return false;
        }

        if(m_map->isObstacle(node->x(), node->y()))
        {
            return false;
        }
        return true;
    }

}  // namespace grstapse