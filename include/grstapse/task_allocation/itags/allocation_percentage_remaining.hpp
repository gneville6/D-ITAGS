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
#include "grstapse/common/search/heuristic_base.hpp"
#include "grstapse/task_allocation/itags/incremental_task_allocation_node.hpp"
#include "grstapse/task_allocation/itags/itags_problem_inputs.hpp"
#include "grstapse/task_allocation/itags/task_allocation_math.hpp"

namespace grstapse
{
    class ItagsProblemInputs;

    /**!
     * Evaluates an allocation by computing the percentage of the desired traits left unsatisfied
     *
     * \see Itags
     *
     * \tparam NodeDeriv The node type that apr will be calculated on
     *
     * \cite An  Interleaved  Approach  to  Trait-Based  Task  Allocation and Scheduling}{2021} G. Neville, A. Messing,
     * H. Ravichandar, S. Hutchinson, S. Chernova, and Harish. (2021). The International Journal of Robotics Research.
     *
     */
    template <typename NodeDeriv = IncrementalTaskAllocationNode>
    requires std::derived_from<NodeDeriv, TaskAllocationNodeBase<NodeDeriv>>
    class AllocationPercentageRemaining : public HeuristicBase<NodeDeriv>
    {
       public:
        //! Constructor
        AllocationPercentageRemaining(const std::shared_ptr<const ItagsProblemInputs>& problem_inputs)
            : m_problem_inputs(problem_inputs)
            , m_desired_traits_sum(problem_inputs->desiredTraitsMatrix().sum())
        {}

        //! \returns The percentage of the desired traits left unsatisfied by the allocation in \p node
        [[nodiscard]] float operator()(const std::shared_ptr<NodeDeriv>& node) const final override
        {
            const Eigen::MatrixXf& allocation = node->allocation();
            const float traits_mismatch_error = traitsMismatchError(*m_problem_inputs->robotTraitsMatrixReduction(),
                                                                    allocation,
                                                                    m_problem_inputs->desiredTraitsMatrix(),
                                                                    m_problem_inputs->teamTraitsMatrix());
            if(m_desired_traits_sum != 0) {
                // ||max(E(A), 0)||_{1, 1} / ||Y||_{1,1}
                node->setAPR(traits_mismatch_error / m_desired_traits_sum);
                return traits_mismatch_error / m_desired_traits_sum;
            }
            else{
                // ||max(E(A), 0)||_{1, 1} / ||Y||_{1,1}
                node->setAPR(0);
                return 0;
            }
        }

       private:
        std::shared_ptr<const ItagsProblemInputs> m_problem_inputs;
        float m_desired_traits_sum;
    };
}  // namespace grstapse