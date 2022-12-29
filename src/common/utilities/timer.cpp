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
#include "grstapse/common/utilities/timer.hpp"

// Local
#include "grstapse/common/utilities/error.hpp"
#include "grstapse/common/utilities/logger.hpp"

namespace grstapse
{
    Timer::Timer()
        : m_running(false)
        , m_time(0.f)
    {}

    void Timer::start()
    {
        if(!m_running)
        {
            m_start_time = std::chrono::steady_clock::now();
            m_running    = true;
        }
        else
        {
            Logger::warn("Timer::start called when already running");
        }
    }

    void Timer::stop()
    {
        if(m_running)
        {
            m_time += timeSinceStart();
            m_running = false;
        }
        else
        {
            Logger::warn("Timer::stop called when not running");
        }
    }

    void Timer::reset()
    {
        if(m_running)
        {
            throw createRuntimeError("Timer still running during reset");
        }

        m_time = 0.f;
    }

    float Timer::get() const
    {
        if(!m_running)
        {
            return m_time;
        }

        return m_time + timeSinceStart();
    }

    float Timer::timeSinceStart() const
    {
        std::chrono::steady_clock::time_point end_time = std::chrono::steady_clock::now();
        return std::chrono::duration<float>(end_time - m_start_time).count();
    }
}  // namespace grstapse