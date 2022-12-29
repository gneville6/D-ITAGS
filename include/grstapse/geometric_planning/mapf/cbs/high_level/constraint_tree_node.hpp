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

// Global
#include <cassert>
#include <iostream>
#include <memory>

// External
#include <robin_hood/robin_hood.hpp>

// Local
#include "grstapse/common/search/search_node_base.hpp"
#include "grstapse/common/utilities/mutable_priority_queue/mutable_priority_queueable.hpp"
#include "grstapse/geometric_planning/mapf/cbs/high_level/constraint_tree_node_base.hpp"
#include "grstapse/geometric_planning/mapf/cbs/low_level/temporal_grid_cell_node.hpp"

namespace grstapse
{
    /**!
     * A node from a constraint tree used by Conflict-Based Search (CBS)
     *
     * \see ConflictBasedSearch
     */
    class ConstraintTreeNode : public ConstraintTreeNodeBase
    {
       public:
        /**!
         * \brief
         *
         * \param num_robots
         * \param cost
         * \param parent
         */
        ConstraintTreeNode(unsigned int num_robots,
                           ConstraintTreeNodeCostType cost,
                           const std::shared_ptr<const ConstraintTreeNodeBase>& parent);

        //! \copydoc ConstraintTreeNodeBase
        void setLowLevelSolution(unsigned int robot,
                                 const std::shared_ptr<const TemporalGridCellNode>& leaf) final override;

        //! \copydoc ConstraintTreeNodeBase
        const std::vector<std::shared_ptr<const TemporalGridCellNode>>& lowLevelSolution(
            unsigned int robot) const final override;

        //! \copydoc ConstraintTreeNodeBase
        void setConstraint(unsigned int robot, const std::shared_ptr<const ConstraintBase>& constraint) final override;

        //! \copydoc ConstraintTreeNodeBase
        [[nodiscard]] const robin_hood::unordered_set<std::shared_ptr<const ConstraintBase>> constraints(
            unsigned int robot) const final override;

       protected:
        //! \copydoc ConstraintTreeNodeBase
        void constraintsInsert(unsigned int robot, robin_hood::unordered_set<std::shared_ptr<const ConstraintBase>>& us)
            const final override;

       private:
        unsigned int m_constraint_robot;                     //!< Which robot the last constraint was applied
        std::shared_ptr<const ConstraintBase> m_constraint;  //!< The last constraint
        std::vector<std::shared_ptr<const TemporalGridCellNode>>
            m_low_level_solution;  //!< The new low level solution for m_constaint_robot after m_constraint was applied
    };

}  // namespace grstapse