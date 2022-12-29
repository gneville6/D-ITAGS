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
#include "grstapse/common/utilities/timer_runner.hpp"

// Local
#include "grstapse/common/utilities/error.hpp"
#include "grstapse/common/utilities/logger.hpp"
#include "grstapse/common/utilities/time_keeper.hpp"
#include "grstapse/common/utilities/timer.hpp"

namespace grstapse
{

    TimerRunner::TimerRunner(const std::string& name)
        : m_name(name)
    {
        TimeKeeper::instance().start(m_name);
    }

    TimerRunner::~TimerRunner()
    {
        TimeKeeper::instance().stop(m_name);
    }
}  // namespace grstapse