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
#include "grstapse/task_allocation/itags/incremental_allocation_edge_applier.hpp"

namespace grstapse::unittests
{
    TEST(IncrementalAllocationEdgeApplier, CanApply)
    {
        IncrementalAllocationEdgeApplier edge_applier(Assignment{2, 2}, nullptr);

        // Allocation matrix is M X N (number_of_tasks X number_of_robots)
        auto root   = std::make_shared<const IncrementalTaskAllocationNode>(MatrixDimensions{.height = 3, .width = 3});
        auto parent = std::make_shared<const IncrementalTaskAllocationNode>(Assignment{.task = 0, .robot = 0}, root);
        auto child  = std::make_shared<const IncrementalTaskAllocationNode>(Assignment{.task = 1, .robot = 1}, parent);

        ASSERT_TRUE(edge_applier.isApplicable(child));

        auto rv = edge_applier.apply(child);
        Eigen::MatrixXf correct_allocation(3, 3);
        correct_allocation.setIdentity();

        ASSERT_EQ(rv->allocation(), correct_allocation);
    }

    TEST(IncrementalAllocationEdgeApplier, CannotApply)
    {
        IncrementalAllocationEdgeApplier edge_applier(Assignment{2, 2}, nullptr);

        // Allocation matrix is M X N (number_of_tasks X number_of_robots)
        auto root   = std::make_shared<const IncrementalTaskAllocationNode>(MatrixDimensions{.height = 3, .width = 3});
        auto parent = std::make_shared<const IncrementalTaskAllocationNode>(Assignment{.task = 0, .robot = 0}, root);
        auto child  = std::make_shared<const IncrementalTaskAllocationNode>(Assignment{.task = 2, .robot = 2}, parent);

        ASSERT_FALSE(edge_applier.isApplicable(child));
    }
}  // namespace grstapse::unittests