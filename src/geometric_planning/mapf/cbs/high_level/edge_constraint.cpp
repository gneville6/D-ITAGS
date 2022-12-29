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
#include "grstapse/geometric_planning/mapf/cbs/high_level/edge_constraint.hpp"

namespace grstapse
{
    EdgeConstraint::EdgeConstraint(unsigned int time,
                                   unsigned int x1,
                                   unsigned int y1,
                                   unsigned int x2,
                                   unsigned int y2)
        : TemporalEdge(time, x1, y1, x2, y2)
    {}

    size_t EdgeConstraint::hash() const
    {
        size_t seed = 0;
        boost::hash_combine(seed, m_time);
        boost::hash_combine(seed, m_x1);
        boost::hash_combine(seed, m_y1);
        boost::hash_combine(seed, m_x2);
        boost::hash_combine(seed, m_y2);
        return seed;
    }
}  // namespace grstapse