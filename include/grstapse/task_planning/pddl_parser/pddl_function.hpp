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
#include <vector>

// Local
#include "grstapse/task_planning/pddl_parser/pddl_base.hpp"

namespace grstapse
{
    class PddlType;
    class PddlVariable;

    //! \brief Container for a predicate or function
    class PddlFunction : public PddlBase
    {
       public:
        /**!
         * Constructor
         *
         * \param name The name of the predicate/function
         * \param parameters A list of parameters for the function
         * \param return_type The type the function returns
         */
        PddlFunction(const std::string& name,
                     const std::vector<PddlVariable>& parameters,
                     std::shared_ptr<const PddlType> return_type);

        //! \returns The name of this predicate/function
        [[nodiscard]] const std::string& name() const;

        //! \returns The identifier of this predicate/function
        [[nodiscard]] unsigned int id() const;

        //! \returns The parameters of this predicate/function
        [[nodiscard]] const std::vector<PddlVariable>& parameters() const;

        //! \returns The return type (boolean if predicate, number if numeric function, and the specific object type if
        //! an object function)
        [[nodiscard]] std::shared_ptr<const PddlType> returnType() const;

        /**!
         * \brief Prints a PDDL representation to the given stream
         *
         * \param os The stream to print to
         * \returns \p os
         */
        std::ostream& print(std::ostream& o) const override;

       private:
        std::string m_name;
        unsigned int m_id;
        std::vector<PddlVariable> m_parameters;
        std::shared_ptr<const PddlType> m_return_type;

        static unsigned int s_next_id;
    };
}  // namespace grstapse