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
#include <memory>

// Local
#include "grstapse/common/search/goal_check_base.hpp"
#include "grstapse/geometric_planning/grid/grid_cell.hpp"

namespace grstapse
{
    /**!
     *
     *
     * \tparam GridCellDeriv A derivative of GridCell
     */
    template <typename GridCellDeriv>
    requires std::derived_from<GridCellDeriv, GridCell>
    // Note: GridCellDerive deriving from NodeBase is checked in HeuristicBase
    class GridCellGoalCheck : public GoalCheckBase<GridCellDeriv>
    {
       public:
        explicit GridCellGoalCheck(const std::shared_ptr<const GridCell>& goal)
            : m_goal(goal)
        {}

        [[nodiscard]] inline bool operator()(const std::shared_ptr<const GridCellDeriv>& node) const final override
        {
            return node->x() == m_goal->x() && node->y() == m_goal->y();
        }

       private:
        std::shared_ptr<const GridCell> m_goal;
    };
}  // namespace grstapse