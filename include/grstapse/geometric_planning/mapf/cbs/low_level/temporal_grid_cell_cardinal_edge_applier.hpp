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
#pragma once

// Local
#include "grstapse/common/search/edge_applier_base.hpp"
#include "grstapse/geometric_planning/mapf/cbs/low_level/temporal_grid_cell_node.hpp"

namespace grstapse
{
    /**!
     * \brief Edge applier for (North, South, East, West, Wait) for MAPF
     */
    class TemporalGridCellCardinalEdgeApplier : public EdgeApplierBase<TemporalGridCellNode>
    {
       public:
        TemporalGridCellCardinalEdgeApplier(int x_diff, int y_diff);

        bool isApplicable(const std::shared_ptr<const TemporalGridCellNode>& base) const override;

        std::shared_ptr<TemporalGridCellNode> apply(
            const std::shared_ptr<const TemporalGridCellNode>& base) const override;

       private:
        int m_x_diff;
        int m_y_diff;
    };

}  // namespace grstapse