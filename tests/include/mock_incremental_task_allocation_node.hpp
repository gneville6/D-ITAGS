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

// External
#include <Eigen/Core>
// Project
#include <grstapse/task_allocation/itags/incremental_task_allocation_node.hpp>

namespace grstapse::mocks
{
    // Forward Declarations

    /**!
     * Mock which replaces the allocation function
     *
     * \see IncrementalTaskAllocationNode
     */
    class MockIncrementalTaskAllocationNode : public IncrementalTaskAllocationNode
    {
       public:
        //! Constructor
        MockIncrementalTaskAllocationNode(const Eigen::MatrixXf& matrix)
            : IncrementalTaskAllocationNode(MatrixDimensions{0, 0})
            , m_matrix(matrix)
        {}

        inline Eigen::MatrixXf allocation() const final override
        {
            return m_matrix;
        }

       private:
        Eigen::MatrixXf m_matrix;
    };

}  // namespace grstapse::mocks