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
#include "grstapse/scheduling/schedule_base.hpp"

// Global
#include <limits>

namespace grstapse {
    ScheduleBase::ScheduleBase()
            : m_makespan(std::numeric_limits<float>::quiet_NaN()) {}

    ScheduleBase::ScheduleBase(
            float makespan,
            const std::vector<std::pair<unsigned int, unsigned int>> &precedence_set_mutex_constraints)
            : m_makespan(makespan), m_precedence_set_mutex_constraints(precedence_set_mutex_constraints) {}


    ScheduleBase::ScheduleBase(ScheduleBase const &toCopy) {
        m_makespan = toCopy.m_makespan;
        m_precedence_set_mutex_constraints = toCopy.m_precedence_set_mutex_constraints;
    }
}  // namespace grstapse