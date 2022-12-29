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
#include "grstapse/task_planning/pddl_parser/pddl_token.hpp"

// Local
#include "grstapse/task_planning/pddl_parser/pddl_symbol.hpp"

namespace grstapse
{
    PddlToken::PddlToken(PddlSymbol symbol)
        : m_symbol(symbol)
        , m_description()
        , m_value()
    {}

    PddlToken::PddlToken(PddlSymbol symbol, const std::string& description)
        : m_symbol(symbol)
        , m_description(description)
        , m_value()
    {}

    PddlToken::PddlToken(float value)
        : m_symbol(PddlSymbol::e_number)
        , m_description()
        , m_value(value)
    {}

    PddlSymbol PddlToken::symbol() const
    {
        return m_symbol;
    }

    const std::string& PddlToken::description() const
    {
        return m_description;
    }

    std::string PddlToken::description()
    {
        return m_description;
    }

    float PddlToken::value() const
    {
        return m_value;
    }
}  // namespace grstapse