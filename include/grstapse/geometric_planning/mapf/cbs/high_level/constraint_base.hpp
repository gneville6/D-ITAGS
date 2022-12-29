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
#include <cstddef>
#include <functional>
#include <memory>

namespace grstapse
{
    /**!
     * Base class for constraints to be used in Conflict-Based Search
     *
     * \note Can only be derived from
     */
    class ConstraintBase
    {
       public:
        //! virtual destructor to make this class polymorphic
        virtual ~ConstraintBase() = default;

        //! hash for unordered_set
        virtual size_t hash() const = 0;

       protected:
        //! protected constructor so this cannot be instantiated directly
        ConstraintBase() = default;
    };
}  // namespace grstapse

namespace std
{
    template <>
    struct hash<std::shared_ptr<grstapse::ConstraintBase>>
    {
        size_t operator()(const std::shared_ptr<grstapse::ConstraintBase>& cb) const
        {
            return cb->hash();
        }
    };
}  // namespace std