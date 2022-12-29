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
#include "grstapse/common/search/a_star/a_star_search_node_base.hpp"
#include "grstapse/geometric_planning/grid/grid_cell.hpp"

namespace grstapse
{
    /**!
     * \brief Wraps a GridCell to make is a node for graph search
     */
    class GridCellNode
        : public GridCell
        , public AStarSearchNodeBase<GridCellNode>
    {
       public:
        /**!
         * \brief
         *
         * \param x
         * \param y
         * \param parent
         */
        GridCellNode(unsigned int x, unsigned int y, const std::shared_ptr<const GridCellNode>& parent = nullptr);

        //! \returns The hash of this node
        [[nodiscard]] unsigned int hash() const override;

       private:
        static unsigned int s_next_id;
    };
}  // namespace grstapse