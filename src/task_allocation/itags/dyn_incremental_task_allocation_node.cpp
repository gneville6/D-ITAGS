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
#include "grstapse/task_allocation/itags/dyn_incremental_task_allocation_node.hpp"

namespace grstapse
{
    template <>
    unsigned int DynIncrementalTaskAllocationNode::TaskAllocationNodeBase::s_next_id = 0;

    DynIncrementalTaskAllocationNode::DynIncrementalTaskAllocationNode(const MatrixDimensions& dimensions)
        : Base(dimensions)
    {}

    DynIncrementalTaskAllocationNode::DynIncrementalTaskAllocationNode(
        const Assignment& assignment,
        const std::shared_ptr<const DynIncrementalTaskAllocationNode>& parent)
        : Base(assignment, parent)
    {}

    unsigned int DynIncrementalTaskAllocationNode::hash() const
    {
        Eigen::MatrixXf alloc = allocation();
        int sub_row           = 1;
        int sub_col           = 1;
        for(int i = 0; i < alloc.rows(); ++i)
        {
            for(int j = 0; j < alloc.cols(); ++j)
            {
                auto h = alloc.coeff(i, j);
                if(alloc.coeff(i, j) != 0)
                {
                    if(sub_row < i + 1)
                    {
                        sub_row = i + 1;
                    }
                    if(sub_col < j + 1)
                    {
                        sub_col = j + 1;
                    }
                }
            }
        }
        auto block = alloc.block(0, 0, sub_row, sub_col);
        return std::hash<Eigen::Block<Eigen::MatrixXf>>()(block);
    }

}  // namespace grstapse