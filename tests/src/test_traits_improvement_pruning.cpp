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
// Global
#include <memory>
// External
#include <Eigen/Core>
#include <gtest/gtest.h>
// Project
#include <grstapse/task_allocation/itags/incremental_task_allocation_node.hpp>
#include <grstapse/task_allocation/itags/robot_traits_matrix_reduction.hpp>
#include <grstapse/task_allocation/itags/traits_improvement_pruning.hpp>
// Mock
#include "mock_itags_problem_inputs.hpp"

namespace grstapse::unittests
{
    TEST(TraitsImprovementPruning, NoPrune)
    {
        auto grstaps_problem_inputs        = std::make_shared<mocks::MockGrstapsProblemInputs>();
        auto robot_traits_matrix_reduction = std::make_shared<RobotTraitsMatrixReduction>();
        grstaps_problem_inputs->setRobotTraitsMatrixReduction(robot_traits_matrix_reduction);

        Eigen::MatrixXf team_traits_matrix(2, 2);
        team_traits_matrix << 1.0f, 1.0f, 1.0f, 1.0f;
        grstaps_problem_inputs->setTeamTraitsMatrix(team_traits_matrix);

        auto problem_inputs = std::make_shared<mocks::MockItagsProblemInputs>(grstaps_problem_inputs);
        Eigen::MatrixXf desired_traits_matrix(2, 2);
        desired_traits_matrix << 2.0f, 2.0f, 2.0f, 2.0f;
        problem_inputs->setDesiredTraitsMatrix(desired_traits_matrix);

        // Allocation matrix is M X N (number_of_tasks X number_of_robots)
        auto root   = std::make_shared<const IncrementalTaskAllocationNode>(MatrixDimensions{.height = 2, .width = 2});
        auto parent = std::make_shared<const IncrementalTaskAllocationNode>(Assignment{.task = 0, .robot = 0}, root);
        auto child  = std::make_shared<const IncrementalTaskAllocationNode>(Assignment{.task = 1, .robot = 0}, parent);
        TraitsImprovementPruning pruning(problem_inputs);
        ASSERT_FALSE(pruning(child));
    }

    TEST(TraitsImprovementPruning, Prune)
    {
        auto grstaps_problem_inputs        = std::make_shared<mocks::MockGrstapsProblemInputs>();
        auto robot_traits_matrix_reduction = std::make_shared<RobotTraitsMatrixReduction>();
        grstaps_problem_inputs->setRobotTraitsMatrixReduction(robot_traits_matrix_reduction);

        Eigen::MatrixXf team_traits_matrix(2, 2);
        team_traits_matrix << 1.0f, 1.0f, 1.0f, 1.0f;
        grstaps_problem_inputs->setTeamTraitsMatrix(team_traits_matrix);

        auto problem_inputs = std::make_shared<mocks::MockItagsProblemInputs>(grstaps_problem_inputs);
        Eigen::MatrixXf desired_traits_matrix(2, 2);
        desired_traits_matrix << 1.0f, 1.0f, 1.0f, 1.0f;
        problem_inputs->setDesiredTraitsMatrix(desired_traits_matrix);

        // Allocation matrix is M X N (number_of_tasks X number_of_robots)
        auto root   = std::make_shared<const IncrementalTaskAllocationNode>(MatrixDimensions{.height = 2, .width = 2});
        auto parent = std::make_shared<const IncrementalTaskAllocationNode>(Assignment{.task = 0, .robot = 0}, root);
        // Assign another robot to a task that is fully allocated
        auto child = std::make_shared<const IncrementalTaskAllocationNode>(Assignment{.task = 0, .robot = 1}, parent);
        TraitsImprovementPruning pruning(problem_inputs);
        ASSERT_TRUE(pruning(child));
    }
}  // namespace grstapse::unittests