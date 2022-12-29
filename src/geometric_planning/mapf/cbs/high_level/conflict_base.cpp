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
#include "grstapse/geometric_planning/mapf/cbs/high_level/conflict_base.hpp"

namespace grstapse
{
    ConflictBase::ConflictBase(const std::array<unsigned int, 2>& agents)
        : m_agents(agents)
    {}

    const std::array<unsigned int, 2>& ConflictBase::agents() const
    {
        return m_agents;
    }

    unsigned int ConflictBase::agent1() const
    {
        return m_agents[0];
    }

    unsigned int ConflictBase::agent2() const
    {
        return m_agents[1];
    }
}  // namespace grstapse