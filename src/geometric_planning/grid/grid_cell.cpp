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
#include "grstapse/geometric_planning/grid/grid_cell.hpp"

// Global
#include <cmath>
// External
#include <boost/functional/hash.hpp>

namespace grstapse
{
    GridCell::GridCell(unsigned int x, unsigned int y)
        : m_x(x)
        , m_y(y)
    {}

    float GridCell::euclideanDistance(const GridCell &rhs) const noexcept
    {
        // Note: split up because of some bug with sqrt
        const float x_diff    = static_cast<float>(m_x) - static_cast<float>(rhs.m_x);
        const float y_diff    = static_cast<float>(m_y) - static_cast<float>(rhs.m_y);
        const float x_diff_sq = powf(x_diff, 2.0f);
        const float y_diff_sq = powf(y_diff, 2.0f);
        const float sq_sum    = x_diff_sq + y_diff_sq;
        return std::sqrt(sq_sum);
    }

    unsigned int GridCell::manhattanDistance(const GridCell &rhs) const noexcept
    {
        return static_cast<unsigned int>(std::abs(static_cast<int>(m_x) - static_cast<int>(rhs.m_x)) +
                                         std::abs(static_cast<int>(m_y) - static_cast<int>(rhs.m_y)));
    }

    std::ostream &operator<<(std::ostream &os, const GridCell &cell)
    {
        return os << "(" << cell.x() << ", " << cell.y() << ")";
    }

    size_t GridCell::hash() const
    {
        size_t seed = 0;
        boost::hash_combine(seed, m_x);
        boost::hash_combine(seed, m_y);
        return seed;
    }

}  // namespace grstapse