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
#include "grstapse/geometric_planning/mapf/cbs/low_level/temporal_grid_cell_goal_check_with_constraints.hpp"

// Local
#include "grstapse/geometric_planning/mapf/cbs/high_level/edge_constraint.hpp"
#include "grstapse/geometric_planning/mapf/cbs/high_level/vertex_constraint.hpp"

namespace grstapse
{
    TemporalGridCellGoalCheckWithConstraints::TemporalGridCellGoalCheckWithConstraints(
        const std::shared_ptr<const GridCell>& goal,
        const robin_hood::unordered_set<std::shared_ptr<const ConstraintBase>>& constraints)
        : m_goal(goal)
        , m_latest_goal_constraint(0)
    {
        for(std::shared_ptr<const ConstraintBase> constraint: constraints)
        {
            auto vertex_constraint = std::dynamic_pointer_cast<const VertexConstraint>(constraint);
            if(vertex_constraint && vertex_constraint->x() == m_goal->x() && vertex_constraint->y() == m_goal->y())
            {
                m_latest_goal_constraint = std::max(m_latest_goal_constraint, vertex_constraint->time());
            }
        }
    }

    bool TemporalGridCellGoalCheckWithConstraints::operator()(
        const std::shared_ptr<const TemporalGridCellNode>& node) const
    {
        return node->x() == m_goal->x() && node->y() == m_goal->y() && node->time() > m_latest_goal_constraint;
    }
}  // namespace grstapse