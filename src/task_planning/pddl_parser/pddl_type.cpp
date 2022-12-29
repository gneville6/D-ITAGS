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
#include "grstapse/task_planning/pddl_parser/pddl_type.hpp"

namespace grstapse
{
    unsigned int PddlType::s_next_id = 0;

    PddlType::PddlType(const std::string& name, std::shared_ptr<const PddlType> parent)
        : m_name(name)
        , m_parent(parent)
        , m_id(s_next_id++)
    {}

    const std::string& PddlType::name() const
    {
        return m_name;
    }

    unsigned int PddlType::id() const
    {
        return m_id;
    }

    std::shared_ptr<const PddlType> PddlType::parent() const
    {
        return m_parent;
    }

    std::ostream& PddlType::print(std::ostream& o) const
    {
        // TODO(Andrew): Implement
        throw std::logic_error("Not Implemented");
        return o;
    }
}  // namespace grstapse