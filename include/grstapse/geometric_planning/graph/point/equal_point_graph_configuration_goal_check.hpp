/*
 * Graphically Recursive Simultaneous Task Allocation, Planning,
 * Scheduling, and Execution
 *
 * Copyright (C) 2020-2022
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
#include "grstapse/common/search/goal_check_base.hpp"
#include "grstapse/geometric_planning/graph/point/point_graph_configuration.hpp"

namespace grstapse
{
    /**!
     * A goal check based on a node being close to the goal configuration
     *
     * \tparam SearchNode
     */
    template <typename SearchNode>
    class EqualPointGraphConfigurationGoalCheck : public GoalCheckBase<SearchNode>
    {
       public:
        //! Constructor
        explicit EqualPointGraphConfigurationGoalCheck(const std::shared_ptr<const PointGraphConfiguration>& goal)
            : m_goal(goal)
        {}

        //! \copydoc GoalCheckBase
        [[nodiscard]] inline bool operator()(const std::shared_ptr<const SearchNode>& node) const final override
        {
            return *node->vertex()->payload() == *m_goal;
        }

       private:
        std::shared_ptr<const PointGraphConfiguration> m_goal;
    };

}  // namespace grstapse