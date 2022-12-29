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
#include <cmath>
#include <concepts>

// Local
#include "grstapse/common/search/heuristic_base.hpp"
#include "grstapse/geometric_planning/grid/grid_cell.hpp"

namespace grstapse
{
    /**!
     * A heuristic for calculating the euclidean distance between a grid cell and the goal grid cell
     *
     * \tparam GridCellDerive A derivative class of GridCell
     */
    template <typename GridCellDerive>
    requires std::derived_from<GridCellDerive, GridCell>
    // Note: GridCellDerive deriving from SearchNodeBase is checked in HeuristicBase
    class GridCellEuclideanDistance : public HeuristicBase<GridCellDerive>
    {
       public:
        /**!
         * Constructor
         *
         * \param goal The grid cell the robot wants to be in
         */
        explicit GridCellEuclideanDistance(const std::shared_ptr<const GridCell>& goal)
            : m_goal(goal)
        {}

        //!\returns The euclidean distance between \p cell and the goal
        [[nodiscard]] inline float operator()(const std::shared_ptr<GridCellDerive>& cell) const final override
        {
            return cell->euclideanDistance(*m_goal);
        }

       private:
        std::shared_ptr<const GridCell> m_goal;
    };
}  // namespace grstapse