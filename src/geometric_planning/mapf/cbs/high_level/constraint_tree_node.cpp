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
#include "grstapse/geometric_planning/mapf/cbs/high_level/constraint_tree_node.hpp"

// Global
#include <cassert>

// Local
#include "grstapse/geometric_planning/mapf/cbs/low_level/temporal_grid_cell_node.hpp"

namespace grstapse
{
    ConstraintTreeNode::ConstraintTreeNode(unsigned int num_robots,
                                           ConstraintTreeNodeCostType cost_type,
                                           const std::shared_ptr<const ConstraintTreeNodeBase>& parent)
        : ConstraintTreeNodeBase(num_robots, cost_type, parent)
    {}

    void ConstraintTreeNode::setLowLevelSolution(unsigned int robot,
                                                 const std::shared_ptr<const TemporalGridCellNode>& leaf)
    {
        assert(robot < m_num_robots && robot == m_constraint_robot);
        m_low_level_solution = trace<TemporalGridCellNode>(leaf);
    }

    const std::vector<std::shared_ptr<const TemporalGridCellNode>>& ConstraintTreeNode::lowLevelSolution(
        unsigned int robot) const
    {
        assert(robot < m_num_robots);
        if(robot == m_constraint_robot)
        {
            return m_low_level_solution;
        }
        return m_parent->lowLevelSolution(robot);
    }

    void ConstraintTreeNode::setConstraint(unsigned int robot, const std::shared_ptr<const ConstraintBase>& constraint)
    {
        assert(robot < m_num_robots);
        m_constraint_robot = robot;
        m_constraint       = constraint;
    }

    const robin_hood::unordered_set<std::shared_ptr<const ConstraintBase>> ConstraintTreeNode::constraints(
        unsigned int robot) const
    {
        robin_hood::unordered_set<std::shared_ptr<const ConstraintBase>> rv;
        if(robot == m_constraint_robot)
        {
            rv.insert(m_constraint);
        }
        m_parent->constraintsInsert(robot, rv);
        return rv;
    }

    void ConstraintTreeNode::constraintsInsert(
        unsigned int robot,
        robin_hood::unordered_set<std::shared_ptr<const ConstraintBase>>& us) const
    {
        if(robot == m_constraint_robot)
        {
            us.insert(m_constraint);
        }
        m_parent->constraintsInsert(robot, us);
    }
}  // namespace grstapse