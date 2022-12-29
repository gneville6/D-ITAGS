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
#include <sstream>

namespace grstapse
{
    //! \brief Abstract base class for a PDDL element
    class PddlBase
    {
       public:
        /**!
         * \brief Prints a PDDL representation to the given stream
         *
         * \param os The stream to print to
         * \returns \p os
         */
        virtual std::ostream& print(std::ostream& o) const = 0;

        //! \returns A string containing a PDDL representation
        std::string str() const;
    };

    /**!
     * \brief Prints a PDDL representation of \p domain to the given stream
     *
     * \param os The stream to print to
     * \returns \p os
     */
    std::ostream& operator<<(std::ostream& os, const PddlBase& pddl);
}  // namespace grstapse