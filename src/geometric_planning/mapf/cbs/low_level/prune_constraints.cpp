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
#include "grstapse/geometric_planning/mapf/cbs/low_level/prune_constraints.hpp"

// External
#include <robin_hood/robin_hood.hpp>

// Local
#include "grstapse/common/utilities/error.hpp"
#include "grstapse/geometric_planning/mapf/cbs/high_level/edge_constraint.hpp"
#include "grstapse/geometric_planning/mapf/cbs/high_level/vertex_constraint.hpp"

namespace grstapse
{
    PruneConstraints::PruneConstraints(
        const robin_hood::unordered_set<std::shared_ptr<const ConstraintBase>>& constraints)
        : m_constraints(constraints)
    {}

    bool PruneConstraints::operator()(const std::shared_ptr<const TemporalGridCellNode>& node) const
    {
        for(std::shared_ptr<const ConstraintBase> constraint: m_constraints)
        {
            auto vertex_constraint = std::dynamic_pointer_cast<const VertexConstraint>(constraint);
            if(vertex_constraint)
            {
                if(prune(node, vertex_constraint))
                {
                    return true;
                }
                continue;
            }

            auto edge_constraint = std::dynamic_pointer_cast<const EdgeConstraint>(constraint);
            if(edge_constraint)
            {
                if(prune(node, edge_constraint))
                {
                    return true;
                }
                continue;
            }
            throw createLogicError("Unknown type of constraint");
        }
        return false;
    }

    bool PruneConstraints::prune(const std::shared_ptr<const TemporalGridCellNode>& node,
                                 const std::shared_ptr<const EdgeConstraint>& constraint) const
    {
        std::shared_ptr<const TemporalGridCellNode> parent = node->parent();
        return EdgeConstraint(parent->time(), parent->x(), parent->y(), node->x(), node->y()) == *constraint;
    }

    bool PruneConstraints::prune(const std::shared_ptr<const TemporalGridCellNode>& node,
                                 const std::shared_ptr<const VertexConstraint>& constraint) const
    {
        return VertexConstraint(node->time(), node->x(), node->y()) == *constraint;
    }
}  // namespace grstapse