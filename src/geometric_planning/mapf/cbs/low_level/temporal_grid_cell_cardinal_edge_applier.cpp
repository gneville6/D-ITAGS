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
#include "grstapse/geometric_planning/mapf/cbs/low_level/temporal_grid_cell_cardinal_edge_applier.hpp"

namespace grstapse
{
    TemporalGridCellCardinalEdgeApplier::TemporalGridCellCardinalEdgeApplier(int x_diff, int y_diff)
        : m_x_diff(x_diff)
        , m_y_diff(y_diff)
    {}

    bool TemporalGridCellCardinalEdgeApplier::isApplicable(
        const std::shared_ptr<const TemporalGridCellNode>& base) const
    {
        if(base->x() + m_x_diff < 0 || base->y() + m_y_diff < 0)
        {
            return false;
        }
        return true;
    }

    std::shared_ptr<TemporalGridCellNode> TemporalGridCellCardinalEdgeApplier::apply(
        const std::shared_ptr<const TemporalGridCellNode>& base) const
    {
        return std::make_shared<TemporalGridCellNode>(base->time() + 1,
                                                      base->x() + m_x_diff,
                                                      base->y() + m_y_diff,
                                                      base);
    }
}  // namespace grstapse