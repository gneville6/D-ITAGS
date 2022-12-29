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

// Local
#include "grstapse/task_planning/pddl_parser/pddl_enums.hpp"

namespace grstapse
{
    /**!
     * Container for the duration of a durative action
     *
     * \note Currently only handle a small subset of what is possible with pddl. (Others can be added if needed)
     * <duration-constraint> ::= (= ?duration <number>)
     * <duration-constraint> ::= ()
     */
    class PddlDuration
    {
       public:
        //! \brief Default Constructor
        PddlDuration();

        /**!
         * Equal constraint constructor
         *
         * @param comparator
         * @param value
         */
        PddlDuration(PddlComparator comparator, float value);

        /**!
         * \returns The comparator
         *
         * \note Current '=' is the only comparator accepted
         */
        PddlComparator comparator() const;

        //! \returns The value
        float value() const;

       private:
        PddlComparator m_comparator;
        float m_value;
    };
}  // namespace grstapse