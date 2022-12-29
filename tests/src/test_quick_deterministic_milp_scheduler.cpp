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
#ifndef NO_MILP

// Global
#    include <fstream>
// External
#    include <fmt/format.h>
#    include <gtest/gtest.h>
// Project
#    include <grstapse/scheduling/milp/deterministic/deterministic_schedule.hpp>
#    include <grstapse/scheduling/milp/deterministic/quick_deterministic_milp_scheduler.hpp>
// Local
#    include "scheduling_setup.hpp"

namespace grstapse::unittests
{
    /**!
     * Basically just tests that it compiles
     */
    TEST(QuickDeterministicMilpScheduler, Simple)
    {
        auto scheduler_problem_inputs =
            createSchedulerProblemInputs(PlanOption::e_complex, AllocationOption::e_complex2, false);
        QuickDeterministicMilpScheduler scheduler(scheduler_problem_inputs);

        auto schedule = std::dynamic_pointer_cast<const DeterministicSchedule>(scheduler.solve());
        ASSERT_TRUE(schedule);
    }
}  // namespace grstapse::unittests

#endif