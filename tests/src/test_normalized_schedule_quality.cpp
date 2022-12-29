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
#include <fmt/format.h>
#include <gtest/gtest.h>
// Project
#include <grstapse/task_allocation/itags/incremental_task_allocation_node.hpp>
#include <grstapse/task_allocation/itags/itags_problem_inputs.hpp>
// Local
#include "mock_normalized_schedule_quality.hpp"

namespace grstapse::unittests
{
    /**!
     * Tests the basic NSQ equation (sched - sched_best) / (sched_worst - sched_best)
     */
    TEST(NormalizedScheduleQuality, EquationCheck)
    {
        // Empty problem inputs
        auto itags_problem_inputs =
            std::shared_ptr<ItagsProblemInputs>(new ItagsProblemInputs(nullptr, {}, {}, {}, 0.0, 2.0f));
        itags_problem_inputs->validate();

        auto nsq = std::make_shared<mocks::MockNormalizedScheduleQuality>(itags_problem_inputs, 1.0);

        // Empty node
        auto node = std::make_shared<IncrementalTaskAllocationNode>(MatrixDimensions{0, 0});

        const float result = nsq->operator()(node);
        ASSERT_FLOAT_EQ(result, 0.5f) << fmt::format("NSQ computation incorrect (true: {0:f}; computed: {1:f})",
                                                     0.5f,
                                                     result);
    }

    /**!
     * Tests computing a set of mutex constraints from an allocation
     */
    TEST(NormalizedScheduleQuality, ComputeMutexConstraints)
    {
        // Empty problem inputs
        auto itags_problem_inputs =
            std::shared_ptr<ItagsProblemInputs>(new ItagsProblemInputs(nullptr, {}, {}, {}, 0.0, 2.0f));
        itags_problem_inputs->validate();

        auto nsq = std::make_shared<mocks::MockNormalizedScheduleQuality>(itags_problem_inputs, 1.0);

        // T0: Empty node
        {
            auto root = std::make_shared<IncrementalTaskAllocationNode>(MatrixDimensions{5, 5});
            const std::set<std::pair<unsigned int, unsigned int>> mutex_constraints =
                nsq->computeMutexConstraints(root);
            ASSERT_TRUE(mutex_constraints.empty())
                << fmt::format("T0: Incorrect number of mutex constraints created (true: {0:d}; computed: {1:d})",
                               0,
                               mutex_constraints.size());
        }

        // T1: One robot with one task allocated
        {
            auto root  = std::make_shared<IncrementalTaskAllocationNode>(MatrixDimensions{5, 5});
            auto node1 = std::make_shared<IncrementalTaskAllocationNode>(Assignment{0, 0}, root);
            const std::set<std::pair<unsigned int, unsigned int>> mutex_constraints =
                nsq->computeMutexConstraints(node1);
            ASSERT_TRUE(mutex_constraints.empty())
                << fmt::format("T1: Incorrect number of mutex constraints created (true: {0:d}; computed: {1:d})",
                               0,
                               mutex_constraints.size());
        }

        // T2: One robot with two tasks allocated
        {
            auto root  = std::make_shared<IncrementalTaskAllocationNode>(MatrixDimensions{5, 5});
            auto node1 = std::make_shared<IncrementalTaskAllocationNode>(Assignment{0, 0}, root);
            auto node2 = std::make_shared<IncrementalTaskAllocationNode>(Assignment{1, 0}, node1);
            const std::set<std::pair<unsigned int, unsigned int>> mutex_constraints =
                nsq->computeMutexConstraints(node2);
            ASSERT_EQ(mutex_constraints.size(), 1)
                << fmt::format("T2: Incorrect number of mutex constraints created (true: {0:d}; computed: {1:d})",
                               1,
                               mutex_constraints.size());
        }

        // T3: Two Robots each with a single allocation to different tasks
        {
            auto root  = std::make_shared<IncrementalTaskAllocationNode>(MatrixDimensions{5, 5});
            auto node1 = std::make_shared<IncrementalTaskAllocationNode>(Assignment{0, 0}, root);
            auto node2 = std::make_shared<IncrementalTaskAllocationNode>(Assignment{1, 1}, node1);
            const std::set<std::pair<unsigned int, unsigned int>> mutex_constraints =
                nsq->computeMutexConstraints(node2);
            ASSERT_EQ(mutex_constraints.size(), 0)
                << fmt::format("T3: Incorrect number of mutex constraints created (true: {0:d}; computed: {1:d})",
                               0,
                               mutex_constraints.size());
        }

        // T4: Two robots each with two allocations to the same two tasks
        {
            auto root  = std::make_shared<IncrementalTaskAllocationNode>(MatrixDimensions{5, 5});
            auto node1 = std::make_shared<IncrementalTaskAllocationNode>(Assignment{0, 0}, root);
            auto node2 = std::make_shared<IncrementalTaskAllocationNode>(Assignment{0, 1}, node1);
            auto node3 = std::make_shared<IncrementalTaskAllocationNode>(Assignment{1, 0}, node2);
            auto node4 = std::make_shared<IncrementalTaskAllocationNode>(Assignment{1, 1}, node3);
            const std::set<std::pair<unsigned int, unsigned int>> mutex_constraints =
                nsq->computeMutexConstraints(node4);
            ASSERT_EQ(mutex_constraints.size(), 1)
                << fmt::format("T4: Incorrect number of mutex constraints created (true: {0:d}; computed: {1:d})",
                               1,
                               mutex_constraints.size());
        }

        // T5: Two robots each with two allocations to different tasks
        {
            auto root  = std::make_shared<IncrementalTaskAllocationNode>(MatrixDimensions{5, 5});
            auto node1 = std::make_shared<IncrementalTaskAllocationNode>(Assignment{0, 0}, root);
            auto node2 = std::make_shared<IncrementalTaskAllocationNode>(Assignment{1, 0}, node1);
            auto node3 = std::make_shared<IncrementalTaskAllocationNode>(Assignment{2, 1}, node2);
            auto node4 = std::make_shared<IncrementalTaskAllocationNode>(Assignment{3, 1}, node3);
            const std::set<std::pair<unsigned int, unsigned int>> mutex_constraints =
                nsq->computeMutexConstraints(node4);
            ASSERT_EQ(mutex_constraints.size(), 2)
                << fmt::format("T5: Incorrect number of mutex constraints created (true: {0:d}; computed: {1:d})",
                               2,
                               mutex_constraints.size());
        }

        // T6: Five robots, five tasks, X shaped allocation
        {
            auto root  = std::make_shared<IncrementalTaskAllocationNode>(MatrixDimensions{5, 5});
            auto node1 = std::make_shared<IncrementalTaskAllocationNode>(Assignment{0, 0}, root);
            auto node2 = std::make_shared<IncrementalTaskAllocationNode>(Assignment{1, 1}, node1);
            auto node3 = std::make_shared<IncrementalTaskAllocationNode>(Assignment{2, 2}, node2);
            auto node4 = std::make_shared<IncrementalTaskAllocationNode>(Assignment{3, 3}, node3);
            auto node5 = std::make_shared<IncrementalTaskAllocationNode>(Assignment{4, 4}, node4);
            auto node6 = std::make_shared<IncrementalTaskAllocationNode>(Assignment{4, 0}, node5);
            auto node7 = std::make_shared<IncrementalTaskAllocationNode>(Assignment{3, 1}, node6);
            auto node8 = std::make_shared<IncrementalTaskAllocationNode>(Assignment{1, 3}, node7);
            auto node9 = std::make_shared<IncrementalTaskAllocationNode>(Assignment{0, 4}, node8);
            const std::set<std::pair<unsigned int, unsigned int>> mutex_constraints =
                nsq->computeMutexConstraints(node9);
            ASSERT_EQ(mutex_constraints.size(), 2)
                << fmt::format("T6: Incorrect number of mutex constraints created (true: {0:d}; computed: {1:d})",
                               2,
                               mutex_constraints.size());
        }

        // T7: Five robots, five tasks
        {
            auto root   = std::make_shared<IncrementalTaskAllocationNode>(MatrixDimensions{5, 5});
            auto node1  = std::make_shared<IncrementalTaskAllocationNode>(Assignment{0, 0}, root);
            auto node2  = std::make_shared<IncrementalTaskAllocationNode>(Assignment{1, 1}, node1);
            auto node3  = std::make_shared<IncrementalTaskAllocationNode>(Assignment{2, 2}, node2);
            auto node4  = std::make_shared<IncrementalTaskAllocationNode>(Assignment{3, 3}, node3);
            auto node5  = std::make_shared<IncrementalTaskAllocationNode>(Assignment{4, 4}, node4);
            auto node6  = std::make_shared<IncrementalTaskAllocationNode>(Assignment{0, 1}, node5);
            auto node7  = std::make_shared<IncrementalTaskAllocationNode>(Assignment{1, 2}, node6);
            auto node8  = std::make_shared<IncrementalTaskAllocationNode>(Assignment{2, 3}, node7);
            auto node9  = std::make_shared<IncrementalTaskAllocationNode>(Assignment{3, 4}, node8);
            auto node10 = std::make_shared<IncrementalTaskAllocationNode>(Assignment{2, 0}, node9);
            auto node11 = std::make_shared<IncrementalTaskAllocationNode>(Assignment{4, 0}, node10);
            const std::set<std::pair<unsigned int, unsigned int>> mutex_constraints =
                nsq->computeMutexConstraints(node11);
            ASSERT_EQ(mutex_constraints.size(), 7)
                << fmt::format("T7: Incorrect number of mutex constraints created (true: {0:d}; computed: {1:d})",
                               7,
                               mutex_constraints.size());
        }
    }
}  // namespace grstapse::unittests