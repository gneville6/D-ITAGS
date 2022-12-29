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
#include "grstapse/scheduling/scheduler_problem_inputs.hpp"

// Local
#include "grstapse/common/utilities/error.hpp"
#include "grstapse/task_allocation/itags/itags_problem_inputs.hpp"

namespace grstapse
{
    SchedulerProblemInputs::SchedulerProblemInputs(
        const std::shared_ptr<const ItagsProblemInputs>& problem_inputs,
        const Eigen::MatrixXf& allocation,
        const std::set<std::pair<unsigned int, unsigned int>>& mutex_constraints)
        : m_itags_problem_inputs(problem_inputs)
        , m_allocation(allocation)
        , m_mutex_constraints(mutex_constraints)
    {}

    void SchedulerProblemInputs::validate() const
    {
        unsigned int num_plan_task = numberOfPlanTasks();
        for(const std::pair<unsigned int, unsigned int>& constraint: m_mutex_constraints)
        {
            if(constraint.first >= num_plan_task || constraint.second >= num_plan_task)
            {
                throw createLogicError("Precedence constraint out of range of the number of plan tasks");
            }
        }
        m_itags_problem_inputs->validate();
    }
}  // namespace grstapse