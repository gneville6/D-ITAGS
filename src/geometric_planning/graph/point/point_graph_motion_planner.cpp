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
#include "grstapse/geometric_planning/graph/point/point_graph_motion_planner.hpp"

// Local
#include "grstapse/common/search/undirected_graph/undirected_graph_path_cost.hpp"
#include "grstapse/common/search/undirected_graph/undirected_graph_successor_generator.hpp"
#include "grstapse/common/utilities/constants.hpp"
#include "grstapse/geometric_planning/graph/point/equal_point_graph_configuration_goal_check.hpp"
#include "grstapse/geometric_planning/graph/point/point_graph_a_star.hpp"
#include "grstapse/geometric_planning/graph/point/point_graph_configuration.hpp"
#include "grstapse/geometric_planning/graph/point/point_graph_configuration_euclidean_distance_heuristic.hpp"
#include "grstapse/geometric_planning/graph/point/point_graph_environment.hpp"
#include "grstapse/geometric_planning/graph/point/point_graph_motion_planning_query_result.hpp"
#include "grstapse/geometric_planning/motion_planner_parameters_base.hpp"

namespace grstapse
{
    PointGraphMotionPlanner::PointGraphMotionPlanner(
        const std::shared_ptr<const MotionPlannerParametersBase>& parameters,
        const std::shared_ptr<PointGraphEnvironment>& graph)
        : GraphMotionPlannerBase(parameters, graph)
        , m_search_parameters(
              new BestFirstSearchParameters{.has_timeout = parameters->timeout > 0.0f,
                                            .timeout     = parameters->timeout,
                                            .timer_name  = constants::k_motion_planning_time + std::string("a_star"),
                                            .save_pruned_nodes = false,
                                            .save_closed_nodes = false})
        , m_astar_functors({
              .pathcost            = std::make_shared<const UndirectedGraphPathCost<SearchNode>>(),
              .heuristic           = nullptr,  // Gets set for each A* search individually
              .successor_generator = std::make_shared<const UndirectedGraphSuccessorGenerator<SearchNode>>(graph),
              .goal_check          = nullptr  // Gets set for each A* search individually
          })
        , m_graph(graph)
    {}

    std::shared_ptr<const MotionPlanningQueryResultBase> PointGraphMotionPlanner::computeMotionPlan(
        const std::shared_ptr<const Species>& species,
        const std::shared_ptr<const ConfigurationBase>& initial_configuration,
        const std::shared_ptr<const ConfigurationBase>& goal_configuration)
    {
        auto ic = std::dynamic_pointer_cast<const PointGraphConfiguration>(initial_configuration);
        auto gc = std::dynamic_pointer_cast<const PointGraphConfiguration>(goal_configuration);

        m_astar_functors.heuristic =
            std::make_shared<const PointGraphConfigurationEuclideanDistanceHeuristic<SearchNode>>(gc);
        m_astar_functors.goal_check = std::make_shared<const EqualPointGraphConfigurationGoalCheck<SearchNode>>(gc);

        PointGraphAStar a_star(m_search_parameters, ic, m_graph, m_astar_functors);
        auto result = a_star.search();
        if(!result.foundGoal())
        {
            return std::make_shared<PointGraphMotionPlanningQueryResult>(MotionPlannerQueryStatus::e_timeout);
        }

        std::vector<std::shared_ptr<PointGraphConfiguration>> path;
        traceApply<SearchNode>(result.goal(),
                               [&path](const std::shared_ptr<const SearchNode>& node)
                               {
                                   path.push_back(node->vertex()->payload());
                               });
        std::reverse(path.begin(), path.end());

        return std::make_shared<PointGraphMotionPlanningQueryResult>(MotionPlannerQueryStatus::e_success, path);
    }
}  // namespace grstapse