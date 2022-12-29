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
#include "grstapse/common/search/a_star/a_star.hpp"
#include "grstapse/geometric_planning/grid/grid_cell_node.hpp"

namespace grstapse
{
    class BestFirstSearchParameters;
    class GridMap;

    /**!
     * An A* search through a 2D grid
     */
    class GridSearch : public AStar<GridCellNode>
    {
        using Base = AStar<GridCellNode>;

       public:
        /**!
         * \brief Constructor
         *
         * \param parameters
         * \param map
         * \param initial
         * \param goal
         */
        GridSearch(const std::shared_ptr<const BestFirstSearchParameters>& parameters,
                   const std::shared_ptr<const GridMap>& map,
                   const std::shared_ptr<const GridCell>& initial,
                   const std::shared_ptr<const GridCell>& goal);

       private:
        /**!
         * \returns A search node representing GridSearch::m_initial
         */
        [[nodiscard]] std::shared_ptr<GridCellNode> createRootNode() override;

        std::shared_ptr<const GridCell> m_initial;
    };
}  // namespace grstapse