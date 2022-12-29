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
#include "grstapse/common/search/goal_check_base.hpp"
#include "grstapse/task_allocation/itags/incremental_task_allocation_node.hpp"
#include "grstapse/task_allocation/itags/itags_problem_inputs.hpp"
#include "grstapse/task_allocation/itags/task_allocation_math.hpp"

namespace grstapse
{
    class ItagsProblemInputs;

    /**!
     * Checks that a task allocation satisfies the desired traits
     *
     * \tparam NodeDeriv The node type that apr will be calculated on
     *
     *
     */
    template <typename NodeDeriv = IncrementalTaskAllocationNode>
    requires std::derived_from<NodeDeriv, TaskAllocationNodeBase<NodeDeriv>>
    class DesiredTraitsCheck : public GoalCheckBase<NodeDeriv>
    {
       public:
        /**!
         * \brief Constructor
         *
         * \param desired_trait_matrix
         * \param robot_trait_matrix
         */
        DesiredTraitsCheck(const std::shared_ptr<const ItagsProblemInputs>& problem_inputs)
            : m_problem_inputs(problem_inputs)
        {}

        //! \returns Whether \p node satisfies the desired traits matrix
        [[nodiscard]] bool operator()(const std::shared_ptr<const NodeDeriv>& node) const final override
        {
            // A
            const Eigen::MatrixXf& allocation = node->allocation();

            // E
            Eigen::MatrixXf traits_mismatch_matrix =
                traitsMismatchMatrix(*m_problem_inputs->robotTraitsMatrixReduction(),
                                     allocation,
                                     m_problem_inputs->desiredTraitsMatrix(),
                                     m_problem_inputs->teamTraitsMatrix());

            // Any positive value means that there are traits that are unsatisfied
            return !(traits_mismatch_matrix.array() > 0).any();
        }

       private:
        std::shared_ptr<const ItagsProblemInputs> m_problem_inputs;
    };
}  // namespace grstapse