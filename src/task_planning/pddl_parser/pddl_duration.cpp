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
#include "grstapse/task_planning/pddl_parser/pddl_duration.hpp"

namespace grstapse
{
    PddlDuration::PddlDuration()
        : m_comparator(PddlComparator::e_eq)
        , m_value(0)
    {}

    PddlDuration::PddlDuration(PddlComparator comparator, float value)
        : m_comparator(comparator)
        , m_value(value)
    {}

    PddlComparator PddlDuration::comparator() const
    {
        return m_comparator;
    }

    float PddlDuration::value() const
    {
        return m_value;
    }
}  // namespace grstapse