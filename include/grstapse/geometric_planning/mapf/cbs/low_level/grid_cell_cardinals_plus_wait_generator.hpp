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
#include "grstapse/common/search/successor_generator_base.hpp"
#include "grstapse/geometric_planning/mapf/cbs/low_level/temporal_grid_cell_node.hpp"

namespace grstapse
{
    // Forward Declarations
    class GridMap;

    /**!
     * Generates the successors for a TemporalGridCellNode (N, S, E, W, Wait)
     */
    class GridCellCardinalsPlusWaitGenerator : public SuccessorGeneratorBase<TemporalGridCellNode>
    {
        using Base = SuccessorGeneratorBase<TemporalGridCellNode>;

       public:
        /**!
         * Constructor
         *
         * \param map
         */
        GridCellCardinalsPlusWaitGenerator(const std::shared_ptr<const GridMap>& map);

       private:
        bool isValidNode(const std::shared_ptr<const TemporalGridCellNode>& node) const final override;

        std::shared_ptr<const GridMap> m_map;
    };
}  // namespace grstapse