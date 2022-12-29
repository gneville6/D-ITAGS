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
#pragma once

// External
//// eigen
#include <Eigen/Core>

// Local
#include "grstapse/common/utilities/matrix_dimensions.hpp"
#include "grstapse/task_allocation/assignment.hpp"
#include "grstapse/task_allocation/itags/normalized_schedule_quality.hpp"
#include "grstapse/task_allocation/itags/task_allocation_node_base.hpp"

namespace grstapse
{
    // Forward Declaration
    class ScheduleBase;

    //! \brief A node that contains an allocation of agents to tasks
    class IncrementalTaskAllocationNode : public TaskAllocationNodeBase<IncrementalTaskAllocationNode>
    {
        using Base = TaskAllocationNodeBase<IncrementalTaskAllocationNode>;

       public:
        /**
         * \brief Constructor for the root node
         * \param dimensions The dimensions of the allocation matrix this node represents
         */
        explicit IncrementalTaskAllocationNode(const MatrixDimensions& dimensions);

        /**!
         * \brief Constructor for any node except the root node
         *
         * \param assignment The last assignment for the allocation matrix this node represents
         * \param parent The parent of this node
         */
        IncrementalTaskAllocationNode(const Assignment& assignment,
                                      const std::shared_ptr<const IncrementalTaskAllocationNode>& parent);

       private:
    };

}  // namespace grstapse