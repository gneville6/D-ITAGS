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
#include "grstapse/geometric_planning/grid/grid_cell_node.hpp"

namespace grstapse
{
    // Forward Declarations
    class GridMap;

    /**
     * Creates the children of a GridCellNode as the GridCells in the four cardinal directions
     */
    class GridCellCardinalsGenerator : public SuccessorGeneratorBase<GridCellNode>
    {
        using Base = SuccessorGeneratorBase<GridCellNode>;

       public:
        //! \brief Constructor
        explicit GridCellCardinalsGenerator(const std::shared_ptr<const GridMap>& map);

       protected:
        //! Checks whether the node is obstacle free and inside the map
        bool isValidNode(const std::shared_ptr<const GridCellNode>& node) const final override;

        std::shared_ptr<const GridMap> m_map;
    };
}  // namespace grstapse