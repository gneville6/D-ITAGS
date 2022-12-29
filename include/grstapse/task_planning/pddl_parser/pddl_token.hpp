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
#include <string>

namespace grstapse
{
    enum class PddlSymbol;

    //! \brief A single token from a pddl file
    class PddlToken
    {
       public:
        //! \brief Constructor for when the token is a known pddl symbol/keyword
        PddlToken(PddlSymbol symbol);

        //! \brief Constructor for when the token is text that is not a keyword
        PddlToken(PddlSymbol symbol, const std::string& description);

        //! \brief Constructor for when the token is a number
        PddlToken(float value);

        //! \returns The symbol
        [[nodiscard]] PddlSymbol symbol() const;

        //! \returns The text when this token does not represent a keyword, symbol, or number
        [[nodiscard]] const std::string& description() const;

        //! \returns The text when this token does not represent a keyword, symbol, or number
        [[nodiscard]] std::string description();

        //! \returns The value when this token is a number
        [[nodiscard]] float value() const;

       private:
        PddlSymbol m_symbol;
        std::string m_description;
        float m_value;
    };
}  // namespace grstapse