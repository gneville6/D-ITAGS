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
#include <robin_hood/robin_hood.hpp>
// Local
#include "grstapse/common/utilities/noncopyable.hpp"
#include "grstapse/common/utilities/timer.hpp"

namespace grstapse
{
    /**!
     * Global singleton that stores the times for named timers
     *
     * \see Timer
     */
    class TimeKeeper : public Noncopyable
    {
       public:
        //! \returns The singleton instance of this class
        static TimeKeeper& instance();

        //! Starts a timer of name \p timer_name
        void start(const std::string& timer_name);

        //! Stops a timer of name \p timer_name
        void stop(const std::string& timer_name);

        //! Resets a timer of name \p timer_name
        void reset(const std::string& timer_name);

        //! \returns The value for a named timer
        [[nodiscard]] float time(const std::string& timer_name) const;

       private:
        //! Constructor
        TimeKeeper() = default;
        robin_hood::unordered_map<std::string, Timer> m_timers;
    };

}  // namespace grstapse