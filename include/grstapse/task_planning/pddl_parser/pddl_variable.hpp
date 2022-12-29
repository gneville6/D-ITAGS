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
#include <memory>
#include <string>

namespace grstapse
{
    class PddlType;

    //! \brief A container for a pddl variable
    class PddlVariable
    {
       public:
        PddlVariable(const std::string& name, std::shared_ptr<const PddlType> type);

        //! \returns The name of the variable
        [[nodiscard]] const std::string& name() const;

        //! \returns The type of the variable
        [[nodiscard]] std::shared_ptr<const PddlType> type() const;

       private:
        std::string m_name;
        std::shared_ptr<const PddlType> m_type;
    };
}  // namespace grstapse