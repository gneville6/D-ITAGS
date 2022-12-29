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
#include <cassert>
#include <vector>

// External
#include <robin_hood/robin_hood.hpp>

// Local
#include "grstapse/geometric_planning/grid/grid_cell.hpp"

namespace grstapse
{
    /**!
     * A 2D grid used for path planning
     */
    class GridMap
    {
       public:
        /**!
         * Constructor
         *
         * \param width Width of the grid
         * \param height Height of the grid
         * \param obstacles List of cells that contain obstacles
         */
        GridMap(unsigned int width, unsigned int height, const robin_hood::unordered_set<GridCell>& obstacles);

        //! \returns The width of the grid
        [[nodiscard]] inline unsigned int width() const noexcept;

        //! \returns The height of the grid
        [[nodiscard]] inline unsigned int height() const noexcept;

        //! \returns Whether the specified cell is an obstacle
        [[nodiscard]] inline bool isObstacle(const GridCell& cell) const;

        //! \returns Whether the specified cell is an obstacle
        [[nodiscard]] inline bool isObstacle(unsigned int x, unsigned int y) const;

       private:
        std::vector<std::vector<bool>> m_map;
    };

    // Inline functions
    unsigned int GridMap::width() const noexcept
    {
        return m_map.size();
    }

    unsigned int GridMap::height() const noexcept
    {
        assert(!m_map.empty());
        return m_map[0].size();
    }

    bool GridMap::isObstacle(const GridCell& cell) const
    {
        return isObstacle(cell.x(), cell.y());
    }

    bool GridMap::isObstacle(unsigned int x, unsigned int y) const
    {
        assert(x < m_map.size() && y < m_map[x].size());
        return m_map[x][y];
    }
}  // namespace grstapse