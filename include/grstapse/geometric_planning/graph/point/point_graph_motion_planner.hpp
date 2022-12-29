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
#include "grstapse/common/search/a_star/a_star_functors.hpp"
#include "grstapse/common/search/best_first_search_parameters.hpp"
#include "grstapse/common/search/undirected_graph/undirected_graph_a_star_search_node.hpp"
#include "grstapse/geometric_planning/graph/graph_motion_planner_base.hpp"
#include "grstapse/geometric_planning/graph/point/point_graph_configuration.hpp"

namespace grstapse
{
    // Forward Declarations
    class PointGraphEnvironment;

    /**!
     *  A motion planner that conducts an A* search through an undirected graph where each vertex represents a point
     *  in 2D space
     */
    class PointGraphMotionPlanner : public GraphMotionPlannerBase
    {
        using SearchNode = UndirectedGraphAStarSearchNode<PointGraphConfiguration>;

       public:
        //! Constructor
        PointGraphMotionPlanner(const std::shared_ptr<const MotionPlannerParametersBase>& parameters,
                                const std::shared_ptr<PointGraphEnvironment>& graph);

       protected:
        //! \copydoc MotionPlannerBase
        std::shared_ptr<const MotionPlanningQueryResultBase> computeMotionPlan(
            const std::shared_ptr<const Species>& species,
            const std::shared_ptr<const ConfigurationBase>& initial_configuration,
            const std::shared_ptr<const ConfigurationBase>& goal_configuration) final override;

        std::shared_ptr<const BestFirstSearchParameters> m_search_parameters;
        AStarFunctors<SearchNode> m_astar_functors;
        std::shared_ptr<PointGraphEnvironment> m_graph;
    };

}  // namespace grstapse