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
#include "grstapse/geometric_planning/mapf/cbs/high_level/constraint_tree_node_base.hpp"

namespace grstapse
{
    //! \brief Root node for a Constraint Tree
    class ConstraintTreeNodeRoot : public ConstraintTreeNodeBase
    {
       public:
        ConstraintTreeNodeRoot(unsigned int num_robot, ConstraintTreeNodeCostType cost_type);

        // \copydoc ConstraintTreeNodeBase
        void setLowLevelSolution(unsigned int robot,
                                 const std::shared_ptr<const TemporalGridCellNode>& leaf) final override;

        // \copydoc ConstraintTreeNodeBase
        const std::vector<std::shared_ptr<const TemporalGridCellNode>>& lowLevelSolution(
            unsigned int robot) const final override;

        // \copydoc ConstraintTreeNodeBase
        void setConstraint(unsigned int robot, const std::shared_ptr<const ConstraintBase>& constraint) final override;

        // \copydoc ConstraintTreeNodeBase
        const robin_hood::unordered_set<std::shared_ptr<const ConstraintBase>> constraints(
            unsigned int robot) const final override;

       protected:
        // \copydoc ConstraintTreeNodeBase
        void constraintsInsert(unsigned int robot, robin_hood::unordered_set<std::shared_ptr<const ConstraintBase>>& us)
            const final override;

       private:
        std::vector<std::vector<std::shared_ptr<const TemporalGridCellNode>>> m_low_level_solutions;
    };

}  // namespace grstapse