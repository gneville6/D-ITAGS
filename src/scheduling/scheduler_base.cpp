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
#include "grstapse/scheduling/scheduler_base.hpp"

// Local
#include "grstapse/common/utilities/constants.hpp"
#include "grstapse/common/utilities/timer_runner.hpp"
#include "grstapse/scheduling/scheduler_parameters.hpp"
#include "grstapse/scheduling/scheduler_problem_inputs.hpp"

namespace grstapse
{
    unsigned int SchedulerBase::s_num_failures = 0;

    SchedulerBase::SchedulerBase(const std::shared_ptr<const SchedulerProblemInputs>& problem_inputs)
        : m_problem_inputs(problem_inputs)
    {}

    std::shared_ptr<const ScheduleBase> SchedulerBase::solve()
    {
        TimerRunner timer_runner(constants::k_scheduling_time);
        return computeSchedule();
    }
    unsigned int SchedulerBase::numFailures()
    {
        return s_num_failures;
    }
}  // namespace grstapse