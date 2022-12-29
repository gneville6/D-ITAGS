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

// External
#include <robin_hood/robin_hood.hpp>

// Local
#include "grstapse/common/search/goal_check_base.hpp"
#include "grstapse/geometric_planning/mapf/cbs/high_level/constraint_base.hpp"
#include "grstapse/geometric_planning/mapf/cbs/low_level/temporal_grid_cell_node.hpp"

namespace grstapse
{
    /**!
     * \brief
     */
    class TemporalGridCellGoalCheckWithConstraints : public GoalCheckBase<TemporalGridCellNode>
    {
       public:
        /**!
         * \brief Constructor
         *
         * \param goal
         * \param constraints
         */
        TemporalGridCellGoalCheckWithConstraints(
            const std::shared_ptr<const GridCell>& goal,
            const robin_hood::unordered_set<std::shared_ptr<const ConstraintBase>>& constraints);

        //! \copydoc GoalCheckBase
        bool operator()(const std::shared_ptr<const TemporalGridCellNode>& node) const final override;

       private:
        std::shared_ptr<const GridCell> m_goal;
        unsigned int m_latest_goal_constraint;
    };

}  // namespace grstapse