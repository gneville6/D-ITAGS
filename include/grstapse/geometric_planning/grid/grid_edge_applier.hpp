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
#include <memory>

// Local
#include "grstapse/common/search/edge_applier_base.hpp"
#include "grstapse/geometric_planning/grid/grid_cell_node.hpp"

namespace grstapse
{
    // Forward Declaration
    class GridMap;

    /**!
     * Applies an edge from a grid cell to one a specified x and y distance away
     */
    class GridEdgeApplier : public EdgeApplierBase<GridCellNode>
    {
       public:
        /**!
         * Constructor
         *
         * \param x_diff The difference in the x index of the result grid cell
         * \param y_diff The difference in the y index of the result grid cell
         */
        GridEdgeApplier(int x_diff, int y_diff);

        //! \returns Whether this edge can be applied to \p base
        bool isApplicable(const std::shared_ptr<const GridCellNode>& base) const override;

        //! \returns The new node from applying this edge to \p base
        std::shared_ptr<GridCellNode> apply(const std::shared_ptr<const GridCellNode>& base) const override;

       private:
        int m_x_diff;
        int m_y_diff;
    };

}  // namespace grstapse