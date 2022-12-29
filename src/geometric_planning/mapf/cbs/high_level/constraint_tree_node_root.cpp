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
#include "grstapse/geometric_planning/mapf/cbs/high_level/constraint_tree_node_root.hpp"

// Global
#include <cassert>

// Local
#include "grstapse/common/utilities/error.hpp"
#include "grstapse/geometric_planning/mapf/cbs/low_level/temporal_grid_cell_node.hpp"

namespace grstapse
{
    ConstraintTreeNodeRoot::ConstraintTreeNodeRoot(unsigned int num_robot, ConstraintTreeNodeCostType cost_type)
        : ConstraintTreeNodeBase(num_robot, cost_type, nullptr)
        , m_low_level_solutions(num_robot)
    {}

    void ConstraintTreeNodeRoot::setLowLevelSolution(unsigned int robot,
                                                     const std::shared_ptr<const TemporalGridCellNode>& leaf)
    {
        assert(robot < m_num_robots);
        m_low_level_solutions[robot] = trace(leaf);
    }

    const std::vector<std::shared_ptr<const TemporalGridCellNode>>& ConstraintTreeNodeRoot::lowLevelSolution(
        unsigned int robot) const
    {
        assert(robot < m_num_robots);
        return m_low_level_solutions[robot];
    }
    void ConstraintTreeNodeRoot::setConstraint(unsigned int robot,
                                               const std::shared_ptr<const ConstraintBase>& constraint)
    {
        throw createLogicError("Cannot set a constraint for the root Constraint Tree Node");
    }

    const robin_hood::unordered_set<std::shared_ptr<const ConstraintBase>> ConstraintTreeNodeRoot::constraints(
        unsigned int robot) const
    {
        return robin_hood::unordered_set<std::shared_ptr<const ConstraintBase>>();
    }
    void ConstraintTreeNodeRoot::constraintsInsert(
        unsigned int robot,
        robin_hood::unordered_set<std::shared_ptr<const ConstraintBase>>& us) const
    {
        return;
    }
}  // namespace grstapse