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

// Global
#include <memory>
// Local
#include "grstapse/common/search/a_star/a_star_search_node_base.hpp"
#include "grstapse/geometric_planning/mapf/cbs/low_level/temporal_grid_cell.hpp"

namespace grstapse
{
    /**!
     * \brief A grid cell paired with time used for search (commonly in Conflict-Base Search)
     */
    class TemporalGridCellNode
        : public TemporalGridCell
        , public AStarSearchNodeBase<TemporalGridCellNode>
    {
       public:
        /**!
         * \brief Constructor
         *
         * \param time
         * \param x The x coordinate of the grid cell
         * \param y The y coordinate of the grid cell
         * \param parent
         */
        TemporalGridCellNode(unsigned int time,
                             unsigned int x,
                             unsigned int y,
                             const std::shared_ptr<const TemporalGridCellNode>& parent = nullptr);

        //! \copydoc SearchNodeBase
        unsigned int hash() const override;

       private:
        static unsigned int s_next_id;
    };
}  // namespace grstapse