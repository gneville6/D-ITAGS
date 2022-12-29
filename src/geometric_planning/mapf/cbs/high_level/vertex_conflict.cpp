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
#include "grstapse/geometric_planning/mapf/cbs/high_level/vertex_conflict.hpp"

// Local
#include "grstapse/geometric_planning/mapf/cbs/high_level/vertex_constraint.hpp"

namespace grstapse
{
    VertexConflict::VertexConflict(const std::array<unsigned int, 2>& agents,
                                   unsigned int time,
                                   unsigned int x,
                                   unsigned int y)
        : ConflictBase(agents)
        , TemporalGridCell(time, x, y)
    {}

    robin_hood::unordered_map<unsigned int, std::shared_ptr<ConstraintBase>> VertexConflict::createConstraints() const
    {
        auto vertex_constraint = std::make_shared<VertexConstraint>(m_time, m_x, m_y);
        return robin_hood::unordered_map<unsigned int, std::shared_ptr<ConstraintBase>>{
            {m_agents[0], vertex_constraint},
            {m_agents[1], vertex_constraint}};
    }
}  // namespace grstapse