/*
 * Graphically Recursive Simultaneous Task Allocation, Planning,
 * Scheduling, and Execution
 *
 * Copyright (C) 2020-2022
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

// External
#include <nlohmann/json.hpp>

namespace grstapse
{

    //! Parameters for a search
    struct SearchParameters
    {
        //! Default Conmstructor
        SearchParameters() = default;

        /**!
         * Constructor
         *
         * \param has_timeout
         * \param timeout
         * \param timer_name
         */
        SearchParameters(bool has_timeout, float timeout, const std::string& timer_name)
            : has_timeout(has_timeout)
            , timeout(timeout)
            , timer_name(timer_name)
        {}

        //! Destructor (for polymorphism)
        virtual ~SearchParameters() = default;

        bool has_timeout;
        float timeout;
        std::string timer_name;
    };

}  // namespace grstapse