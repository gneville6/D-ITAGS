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

// Global
#include <fstream>
// External
#include <gtest/gtest.h>
// Project
#include <grstapse/common/search/best_first_search_parameters.hpp>
#include <grstapse/common/search/undirected_graph/undirected_graph_a_star_search_node.hpp>
#include <grstapse/common/search/undirected_graph/undirected_graph_path_cost.hpp>
#include <grstapse/common/search/undirected_graph/undirected_graph_successor_generator.hpp>
#include <grstapse/common/utilities/custom_json_conversions.hpp>
#include <grstapse/geometric_planning/graph/point/equal_point_graph_configuration_goal_check.hpp>
#include <grstapse/geometric_planning/graph/point/point_graph_a_star.hpp>
#include <grstapse/geometric_planning/graph/point/point_graph_configuration_euclidean_distance_heuristic.hpp>
#include <grstapse/geometric_planning/graph/point/point_graph_environment.hpp>

namespace grstapse::unittests
{
    TEST(PointGraphAStar, Simple)
    {
        using SearchNode = UndirectedGraphAStarSearchNode<PointGraphConfiguration>;

        auto search_parameters = std::shared_ptr<const BestFirstSearchParameters>(
            new BestFirstSearchParameters{.has_timeout       = false,
                                          .timeout           = 0.0f,
                                          .timer_name        = "point_graph_a_star",
                                          .save_pruned_nodes = false,
                                          .save_closed_nodes = false});
        std::ifstream in("data/geometric_planning/environments/point_graph.json");
        nlohmann::json j;
        in >> j;

        auto graph                 = j.get<std::shared_ptr<PointGraphEnvironment>>();
        auto initial_configuration = std::make_shared<const PointGraphConfiguration>(0, 0.0f, 0.0f);
        auto goal_configuration    = std::make_shared<const PointGraphConfiguration>(18, 4.0f, 4.0f);

        AStarFunctors<SearchNode> functors{
            .pathcost  = std::make_shared<const UndirectedGraphPathCost<SearchNode>>(),
            .heuristic = std::make_shared<const PointGraphConfigurationEuclideanDistanceHeuristic<SearchNode>>(
                goal_configuration),
            .successor_generator = std::make_shared<const UndirectedGraphSuccessorGenerator<SearchNode>>(graph),
            .goal_check =
                std::make_shared<const EqualPointGraphConfigurationGoalCheck<SearchNode>>(goal_configuration)};

        PointGraphAStar a_star(search_parameters, initial_configuration, graph, functors);
        auto results = a_star.search();
        auto path    = trace<SearchNode>(results.goal());
        ASSERT_TRUE(results.foundGoal());
        ASSERT_EQ(path.size(), 9);
    }

}  // namespace grstapse::unittests