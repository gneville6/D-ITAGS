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
#include <concepts>

// Local
#include "grstapse/common/utilities/custom_concepts.hpp"

namespace grstapse
{
    /**!
     * \brief Interface for an object that can be put into MutablePriorityQueue
     *
     * \tparam PriorityType
     */
    template <typename PriorityType>
    requires LessThanComparable<PriorityType>
    class MutablePriorityQueueable
    {
       public:
        [[nodiscard]] virtual PriorityType priority() const = 0;
    };
}  // namespace grstapse