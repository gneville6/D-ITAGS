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
#include "grstapse/geometric_planning/grid/grid_edge_applier.hpp"

namespace grstapse
{
    GridEdgeApplier::GridEdgeApplier(int x_diff, int y_diff)
        : m_x_diff(x_diff)
        , m_y_diff(y_diff)
    {}

    bool GridEdgeApplier::isApplicable(const std::shared_ptr<const GridCellNode>& base) const
    {
        if(base->x() + m_x_diff < 0 || base->y() + m_y_diff < 0)
        {
            return false;
        }
        return true;
    }

    std::shared_ptr<GridCellNode> GridEdgeApplier::apply(const std::shared_ptr<const GridCellNode>& base) const
    {
        return std::make_shared<GridCellNode>(static_cast<unsigned int>(base->x() + m_x_diff),
                                              static_cast<unsigned int>(base->y() + m_y_diff),
                                              base);
    }
}  // namespace grstapse