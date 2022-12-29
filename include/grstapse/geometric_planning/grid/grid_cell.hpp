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
#include <ostream>
#include <tuple>
#include <utility>

namespace grstapse
{
    /**!
     * A cell in a grid
     */
    class GridCell
    {
       public:
        /**!
         * Constructor
         *
         * \param x The x coordinate of the grid cell
         * \param y The y coordinate of the grid cell
         */
        GridCell(unsigned int x, unsigned y);

        //! \returns The x coordinate of this grid cell
        [[nodiscard]] inline unsigned int x() const noexcept;

        //! \returns The x coordinate of this grid cell
        [[nodiscard]] inline unsigned int y() const noexcept;

        //! Equality Operator
        [[nodiscard]] inline bool operator==(const GridCell& rhs) const noexcept;

        //! Inequality Operator
        [[nodiscard]] inline bool operator!=(const GridCell& rhs) const noexcept;

        //! \returns The euclidean distance to \p rhs
        [[nodiscard]] float euclideanDistance(const GridCell& rhs) const noexcept;

        //! \returns The manhattan distance to \p rhs
        [[nodiscard]] unsigned int manhattanDistance(const GridCell& rhs) const noexcept;

        //! Stream Operator
        friend std::ostream& operator<<(std::ostream& os, const GridCell& cell);

        //! \returns A hash of this GridCell
        size_t hash() const;

       protected:
        unsigned int m_x;
        unsigned int m_y;
    };

    // Inline functions
    unsigned int GridCell::x() const noexcept
    {
        return m_x;
    }
    unsigned int GridCell::y() const noexcept
    {
        return m_y;
    }
    bool GridCell::operator==(const GridCell& rhs) const noexcept
    {
        return std::tie(m_x, m_y) == std::tie(rhs.m_x, rhs.m_y);
    }
    bool GridCell::operator!=(const GridCell& rhs) const noexcept
    {
        return !(*this == rhs);
    }

}  // namespace grstapse

namespace std
{
    template <>
    struct hash<grstapse::GridCell>
    {
        size_t operator()(const grstapse::GridCell& gc) const
        {
            return gc.hash();
        }
    };
}  // namespace std