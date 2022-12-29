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
#include <tuple>

namespace grstapse
{
    /**!
     *
     */
    class TemporalEdge
    {
       public:
        TemporalEdge(unsigned int time, unsigned int x1, unsigned int y1, unsigned int x2, unsigned int y2);

        //! \returns The time for which this edge is constrained
        [[nodiscard]] inline unsigned int time() const noexcept;

        //! \returns The x component of one of the vertices of the edge
        [[nodiscard]] inline unsigned int x1() const noexcept;

        //! \returns The y component of one of the vertices of the edge
        [[nodiscard]] inline unsigned int y1() const noexcept;

        //! \returns The x component of the other vertex of the edge
        [[nodiscard]] inline unsigned int x2() const noexcept;

        //! \returns The y component of the other vertex of the edge
        [[nodiscard]] inline unsigned int y2() const noexcept;

        //! Equals operator
        [[nodiscard]] inline bool operator==(const TemporalEdge& rhs) const noexcept;

        //! Not equals operator
        [[nodiscard]] inline bool operator!=(const TemporalEdge& rhs) const noexcept;

       protected:
        unsigned int m_time;
        unsigned int m_x1;
        unsigned int m_y1;
        unsigned int m_x2;
        unsigned int m_y2;
    };

    // Inline functions
    unsigned int TemporalEdge::time() const noexcept
    {
        return m_time;
    }
    unsigned int TemporalEdge::x1() const noexcept
    {
        return m_x1;
    }
    unsigned int TemporalEdge::y1() const noexcept
    {
        return m_y1;
    }
    unsigned int TemporalEdge::x2() const noexcept
    {
        return m_x2;
    }
    unsigned int TemporalEdge::y2() const noexcept
    {
        return m_y2;
    }
    bool TemporalEdge::operator==(const TemporalEdge& rhs) const noexcept
    {
        return std::tie(m_time, m_x1, m_y1, m_x2, m_y2) == std::tie(rhs.m_time, rhs.m_x1, rhs.m_y1, rhs.m_x2, rhs.m_y2);
    }
    bool TemporalEdge::operator!=(const TemporalEdge& rhs) const noexcept
    {
        return std::tie(m_time, m_x1, m_y1, m_x2, m_y2) != std::tie(rhs.m_time, rhs.m_x1, rhs.m_y1, rhs.m_x2, rhs.m_y2);
    }
}  // namespace grstapse