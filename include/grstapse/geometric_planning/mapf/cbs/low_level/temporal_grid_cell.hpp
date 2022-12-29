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
#include "grstapse/geometric_planning/grid/grid_cell.hpp"

namespace grstapse
{
    class TemporalGridCell : public GridCell
    {
       public:
        TemporalGridCell(unsigned int time, unsigned int x, unsigned int y);

        //! \returns The time associated with this Temporal GridCell
        [[nodiscard]] inline unsigned int time() const noexcept;

        //! Equals operator
        [[nodiscard]] inline bool operator==(const TemporalGridCell& rhs) const noexcept;

        //! Not equals operator
        [[nodiscard]] inline bool operator!=(const TemporalGridCell& rhs) const noexcept;

       protected:
        unsigned int m_time;
    };

    // Inline functions
    unsigned int TemporalGridCell::time() const noexcept
    {
        return m_time;
    }
    bool TemporalGridCell::operator==(const TemporalGridCell& rhs) const noexcept
    {
        return std::tie(m_time, m_x, m_y) == std::tie(rhs.m_time, rhs.m_x, rhs.m_y);
    }
    bool TemporalGridCell::operator!=(const TemporalGridCell& rhs) const noexcept
    {
        return std::tie(m_time, m_x, m_y) != std::tie(rhs.m_time, rhs.m_x, rhs.m_y);
    }
}  // namespace grstapse