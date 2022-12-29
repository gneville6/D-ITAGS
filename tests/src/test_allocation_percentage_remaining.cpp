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
// Global
#include <memory>
// External
#include <Eigen/Core>
#include <gtest/gtest.h>
// Project
#include <grstapse/task_allocation/itags/allocation_percentage_remaining.hpp>
#include <grstapse/task_allocation/itags/itags_problem_inputs.hpp>
#include <grstapse/task_allocation/itags/robot_traits_matrix_reduction.hpp>
// Local
#include "mock_grstaps_problem_inputs.hpp"
#include "mock_incremental_task_allocation_node.hpp"

namespace grstapse::unittests
{
    /**!
     * Tests whether APR works correctly
     */
    TEST(AllocationPercentageRemaining, Simple)
    {
        // Matrix Multiply
        auto robot_traits_matrix_reduction = std::make_shared<RobotTraitsMatrixReduction>();
        // Team Traits Matrix
        Eigen::MatrixXf team_traits_matrix(2, 2);
        team_traits_matrix << 1.0f, 2.0f, 3.0f, 4.0f;  // [ [1 2] [3 4] ]
        // Desired Traits Matrix
        Eigen::MatrixXf desired_traits_matrix(1, 2);
        desired_traits_matrix << 2.0f, 4.0f;  // [2 4]

        auto grstaps_problem_inputs = std::make_shared<mocks::MockGrstapsProblemInputs>();
        grstaps_problem_inputs->setRobotTraitsMatrixReduction(robot_traits_matrix_reduction);
        grstaps_problem_inputs->setTeamTraitsMatrix(team_traits_matrix);
        auto itags_problem_inputs = std::shared_ptr<const ItagsProblemInputs>(
            new ItagsProblemInputs(grstaps_problem_inputs, {}, {}, desired_traits_matrix, 0.0, 0.0));
        itags_problem_inputs->validate();

        AllocationPercentageRemaining apr(itags_problem_inputs);

        // Allocation
        Eigen::MatrixXf allocation(1, 2);
        allocation << 1.0f, 0.0f;  // [1 0]

        auto node = std::make_shared<mocks::MockIncrementalTaskAllocationNode>(allocation);
        ASSERT_FLOAT_EQ(apr(node), 0.5);
    }
}  // namespace grstapse::unittests