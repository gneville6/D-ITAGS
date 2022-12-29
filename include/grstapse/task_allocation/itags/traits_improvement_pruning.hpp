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
#include "grstapse/common/search/pruning_method_base.hpp"
#include "grstapse/task_allocation/itags/incremental_task_allocation_node.hpp"
#include "grstapse/task_allocation/itags/itags_problem_inputs.hpp"
#include "grstapse/task_allocation/itags/task_allocation_math.hpp"
#include "grstapse/task_allocation/itags/task_allocation_node_base.hpp"

namespace grstapse
{
    // Forward Declarations
    class ItagsProblemInputs;

    /**!
     *  Prunes a node if it does not improve the traits mismatch error with respect to its parent node
     *
     * \tparam NodeDeriv The node type that apr will be calculated on
     *
     *
     */
    template <typename NodeDeriv = IncrementalTaskAllocationNode>
    requires std::derived_from<NodeDeriv, TaskAllocationNodeBase<NodeDeriv>>
    class TraitsImprovementPruning : public PruningMethodBase<NodeDeriv>
    {
       public:
        //! Constructor
        explicit TraitsImprovementPruning(const std::shared_ptr<const ItagsProblemInputs>& problem_inputs)
            : m_problem_inputs(problem_inputs)
        {}

        //! \copydoc PruningMethodBase
        [[nodiscard]] virtual bool operator()(const std::shared_ptr<const NodeDeriv>& node) const final override
        {
            Eigen::MatrixXf potential_successor_matrix = node->allocation();

            Eigen::MatrixXf parent_matrix                                              = potential_successor_matrix;
            parent_matrix(node->lastAssigment()->task, node->lastAssigment()->robot) = 0.0f;

            const float potential_successor_error = traitsMismatchError(*m_problem_inputs->robotTraitsMatrixReduction(),
                                                                        potential_successor_matrix,
                                                                        m_problem_inputs->desiredTraitsMatrix(),
                                                                        m_problem_inputs->teamTraitsMatrix());

            const float parent_error = traitsMismatchError(*m_problem_inputs->robotTraitsMatrixReduction(),
                                                           parent_matrix,
                                                           m_problem_inputs->desiredTraitsMatrix(),
                                                           m_problem_inputs->teamTraitsMatrix());

            return potential_successor_error >= parent_error;
        }


       private:
        std::shared_ptr<const ItagsProblemInputs> m_problem_inputs;
    };

}  // namespace grstapse