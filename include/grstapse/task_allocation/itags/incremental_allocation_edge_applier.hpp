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

// Local
#include "grstapse/common/search/edge_applier_base.hpp"
#include "grstapse/task_allocation/itags/incremental_task_allocation_node.hpp"
#include "grstapse/task_allocation/assignment.hpp"

namespace grstapse
{
    class ItagsProblemInputs;

    /**!
     * An edge apply for incremental task allocation nodes
     *
     * \tparam NodeDeriv The node type that apr will be calculated on
     *
     *
     */
    template <typename NodeDeriv = IncrementalTaskAllocationNode>
    requires std::derived_from<NodeDeriv, TaskAllocationNodeBase<NodeDeriv>>
    class IncrementalAllocationEdgeApplier : public EdgeApplierBase<NodeDeriv>
    {
       public:
        /**!
         * Constructor
         *
         * \param assignment The assignment this edge represents
         * \param problem_inputs Inputs to the task allocation problem
         */
        IncrementalAllocationEdgeApplier(const Assignment& assignment,
                                         const std::shared_ptr<const ItagsProblemInputs>& problem_inputs)
            : m_assignment(assignment)
            , m_problem_inputs(problem_inputs)
            {}

        //! \returns Whether the edge this edge applier represents can be added to \p base
        [[nodiscard]] bool isApplicable(
            const std::shared_ptr<const NodeDeriv>& base) const final override
        {
            // If the assignment has already been added then ignore
            std::shared_ptr<const NodeDeriv> parent;
            for(parent = base; parent != nullptr; parent = parent->parent())
            {
                if(const std::optional<Assignment>& last_assignment = parent->lastAssigment();
                   last_assignment.has_value() && last_assignment.value() == m_assignment)
                {
                    return false;
                }
            }

            return true;
        }

        //! \returns The succeeding node if this edge applier can be applied, nullptr otherwise
        [[nodiscard]] std::shared_ptr<NodeDeriv> apply(
            const std::shared_ptr<const NodeDeriv>& base) const final override
        {
            return std::make_shared<NodeDeriv>(m_assignment, base);
        }

       private:
        Assignment m_assignment;
        std::shared_ptr<const ItagsProblemInputs> m_problem_inputs;
    };
}  // namespace grstapse