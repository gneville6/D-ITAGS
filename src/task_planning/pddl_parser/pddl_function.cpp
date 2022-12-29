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
#include "grstapse/task_planning/pddl_parser/pddl_function.hpp"

// Local
#include "grstapse/task_planning/pddl_parser/pddl_variable.hpp"

namespace grstapse
{
    unsigned int PddlFunction::s_next_id = 0;

    PddlFunction::PddlFunction(const std::string& name,
                               const std::vector<PddlVariable>& parameters,
                               std::shared_ptr<const PddlType> return_type)
        : m_name(name)
        , m_id(s_next_id++)
        , m_parameters(parameters)
        , m_return_type(return_type)
    {}

    const std::string& PddlFunction::name() const
    {
        return m_name;
    }

    unsigned int PddlFunction::id() const
    {
        return m_id;
    }

    const std::vector<PddlVariable>& PddlFunction::parameters() const
    {
        return m_parameters;
    }

    std::shared_ptr<const PddlType> PddlFunction::returnType() const
    {
        return m_return_type;
    }

    std::ostream& PddlFunction::print(std::ostream& o) const
    {
        // TODO(Andrew): Implement
        throw std::logic_error("Not Implemented");
        return o;
    }
}  // namespace grstapse