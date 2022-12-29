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
#include <grstapse/task_allocation/itags/itags_problem_inputs.hpp>
// Local
#include "mock_grstaps_problem_inputs.hpp"

namespace grstapse::mocks
{
    /**!
     * Mock to interface with some of the private functions for testing
     */
    class MockItagsProblemInputs : public ItagsProblemInputs
    {
       public:
        //! Constructor
        MockItagsProblemInputs() = default;

        MockItagsProblemInputs(const std::shared_ptr<const MockGrstapsProblemInputs>& grstaps_problem_inputs,
                               const std::vector<unsigned int>& plan_task_indicies                     = {},
                               const std::multimap<unsigned int, unsigned int>& precedence_constraints = {},
                               const Eigen::MatrixXf& desired_traits_matrix                            = {},
                               const float schedule_best_makespan                                      = 0.0f,
                               const float schedule_worst_makespan = std::numeric_limits<float>::infinity())
            : ItagsProblemInputs(grstaps_problem_inputs,
                                 plan_task_indicies,
                                 precedence_constraints,
                                 desired_traits_matrix,
                                 schedule_best_makespan,
                                 schedule_worst_makespan)
        {}

        void setDesiredTraitsMatrix(const Eigen::MatrixXf& matrix)
        {
            m_desired_traits_matrix = matrix;
        }

        using ItagsProblemInputs::loadTasks;
    };

}  // namespace grstapse::mocks