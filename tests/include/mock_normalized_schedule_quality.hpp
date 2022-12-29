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

// Project
#include <grstapse/task_allocation/itags/incremental_task_allocation_node.hpp>
#include <grstapse/task_allocation/itags/normalized_schedule_quality.hpp>

namespace grstapse::mocks
{
    /**!
     * Mock which replaces the computeMakespan function
     *
     * \see NormalizedScheduleQuality
     */
    class MockNormalizedScheduleQuality : public NormalizedScheduleQuality<IncrementalTaskAllocationNode>
    {
       public:
        //! Constructor
        MockNormalizedScheduleQuality(const std::shared_ptr<const ItagsProblemInputs>& problem_inputs,
                                      const float makespan);

        using NormalizedScheduleQuality::computeMutexConstraints;

       private:
        [[nodiscard]] virtual float computeMakespan(
            const std::shared_ptr<IncrementalTaskAllocationNode>& node) const final override;

        float m_makespan;
    };
}  // namespace grstapse::mocks