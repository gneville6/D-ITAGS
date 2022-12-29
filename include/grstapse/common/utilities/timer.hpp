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

// Global
#include <chrono>

namespace grstapse
{
    //! \brief A simple timer
    class Timer
    {
       public:
        //! \brief Constructor
        Timer();

        //! \brief Starts the timer
        void start();

        //! \brief Stops the timer
        void stop();

        //! \brief Resets the timer
        void reset();

        //! \returns The value from the timer in seconds
        [[nodiscard]] float get() const;

       private:
        [[nodiscard]] float timeSinceStart() const;

        bool m_running;
        std::chrono::steady_clock::time_point m_start_time;
        float m_time;  //!< seconds
    };
}  // namespace grstapse