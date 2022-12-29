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
#include <nlohmann/json.hpp>

// Project
#include <grstapse/geometric_planning/mapf/cbs/conflict_based_search.hpp>
#include <grstapse/geometric_planning/mapf/cbs/conflict_based_search_parameters.hpp>
#include <grstapse/geometric_planning/mapf/cbs/conflict_based_search_statistics.hpp>
#include <grstapse/geometric_planning/mapf/cbs/high_level/conflict_base.hpp>
#include <grstapse/geometric_planning/mapf/cbs/high_level/constraint_tree_node.hpp>

namespace grstapse::unittests {
    std::shared_ptr<const ConflictBasedSearchParameters> readParametersFromJson(const std::string &filepath) {
        std::ifstream input(filepath);
        nlohmann::json data;
        input >> data;
        const nlohmann::json &json_map = data["map"];

        const unsigned int width = json_map["dimensions"][0].get<unsigned int>();
        const unsigned int height = json_map["dimensions"][1].get<unsigned int>();
        robin_hood::unordered_set<GridCell> obstacles;
        for (const nlohmann::json &obstacle: json_map["obstacles"]) {
            obstacles.insert(GridCell(obstacle[0].get<unsigned int>(), obstacle[1].get<unsigned int>()));
        }
        auto map = std::make_shared<const GridMap>(width, height, obstacles);

        const nlohmann::json &json_agents = data["agents"];
        const unsigned int num_agents = json_agents.size();

        std::vector<std::shared_ptr<const GridCell>> initial_states;
        initial_states.reserve(num_agents);
        std::vector<std::shared_ptr<const GridCell>> goal_states;
        goal_states.reserve(num_agents);

        for (const nlohmann::json &agents: json_agents) {
            initial_states.push_back(std::make_shared<const GridCell>(agents["start"][0].get<unsigned int>(),
                                                                      agents["start"][1].get<unsigned int>()));
            goal_states.push_back(std::make_shared<const GridCell>(agents["goal"][0].get<unsigned int>(),
                                                                   agents["goal"][1].get<unsigned int>()));
        }

        return std::make_shared<const ConflictBasedSearchParameters>(ConstraintTreeNodeCostType::e_makespan,
                                                                     map,
                                                                     initial_states,
                                                                     goal_states,
                                                                     "cbs_high_level",
                                                                     "cbs_low_level");
    }

    TEST(CBS, atgoal) {
        std::shared_ptr<const ConflictBasedSearchParameters> parameters =
                readParametersFromJson("./data/geometric_planning/mapf/at_goal.json");
        ConflictBaseSearch cbs(parameters);

        SearchResults<ConstraintTreeNodeBase, ConflictBasedSearchStatistics> result = cbs.search();
        ASSERT_TRUE(result.foundGoal());
        std::shared_ptr<const ConstraintTreeNodeBase> goal = result.goal();
        std::shared_ptr<const ConflictBasedSearchStatistics> statistics = result.statistics();
        ASSERT_EQ(statistics->numberOfHighLevelNodesGenerated(), 1);
        ASSERT_EQ(statistics->numberOfHighLevelNodesEvaluated(), 0);
        ASSERT_EQ(statistics->numberOfHighLevelNodesExpanded(), 0);
    }

    TEST(CBS, circle) {
        std::shared_ptr<const ConflictBasedSearchParameters> parameters =
                readParametersFromJson("data/geometric_planning/mapf/circle.json");
        ConflictBaseSearch cbs(parameters);

        SearchResults<ConstraintTreeNodeBase, ConflictBasedSearchStatistics> result = cbs.search();
        ASSERT_TRUE(result.foundGoal());
        std::shared_ptr<const ConstraintTreeNodeBase> goal = result.goal();
        std::shared_ptr<const ConflictBasedSearchStatistics> statistics = result.statistics();
        ASSERT_EQ(statistics->numberOfHighLevelNodesGenerated(), 1);
        ASSERT_EQ(statistics->numberOfHighLevelNodesEvaluated(), 0);
        ASSERT_EQ(statistics->numberOfHighLevelNodesExpanded(), 0);
    }

    TEST(CBS, simple1) {
        std::shared_ptr<const ConflictBasedSearchParameters> parameters =
                readParametersFromJson("data/geometric_planning/mapf/simple1.json");
        ConflictBaseSearch cbs(parameters);

        SearchResults<ConstraintTreeNodeBase, ConflictBasedSearchStatistics> result = cbs.search();
        ASSERT_TRUE(result.foundGoal());
        std::shared_ptr<const ConstraintTreeNodeBase> goal = result.goal();
        std::shared_ptr<const ConflictBasedSearchStatistics> statistics = result.statistics();
        ASSERT_EQ(goal->getFirstConflict(), nullptr);
    }

    TEST(CBS, simple1b) {
        std::shared_ptr<const ConflictBasedSearchParameters> parameters =
                readParametersFromJson("data/geometric_planning/mapf/simple1b.json");
        ConflictBaseSearch cbs(parameters);

        SearchResults<ConstraintTreeNodeBase, ConflictBasedSearchStatistics> result = cbs.search();
        ASSERT_TRUE(result.foundGoal());
        std::shared_ptr<const ConstraintTreeNodeBase> goal = result.goal();
        std::shared_ptr<const ConflictBasedSearchStatistics> statistics = result.statistics();
        ASSERT_EQ(goal->getFirstConflict(), nullptr);
    }

    TEST(CBS, swap2) {
        std::shared_ptr<const ConflictBasedSearchParameters> parameters =
                readParametersFromJson("data/geometric_planning/mapf/swap2.json");
        ConflictBaseSearch cbs(parameters);

        SearchResults<ConstraintTreeNodeBase, ConflictBasedSearchStatistics> result = cbs.search();
        ASSERT_TRUE(result.foundGoal());
        std::shared_ptr<const ConstraintTreeNodeBase> goal = result.goal();
        std::shared_ptr<const ConflictBasedSearchStatistics> statistics = result.statistics();
        ASSERT_EQ(goal->getFirstConflict(), nullptr);
    }

    TEST(CBS, swap4) {
        std::shared_ptr<const ConflictBasedSearchParameters> parameters =
                readParametersFromJson("data/geometric_planning/mapf/swap4.json");
        ConflictBaseSearch cbs(parameters);

        SearchResults<ConstraintTreeNodeBase, ConflictBasedSearchStatistics> result = cbs.search();
        ASSERT_TRUE(result.foundGoal());
        std::shared_ptr<const ConstraintTreeNodeBase> goal = result.goal();
        std::shared_ptr<const ConflictBasedSearchStatistics> statistics = result.statistics();
        ASSERT_EQ(goal->getFirstConflict(), nullptr);
    }
}  // namespace grstapse::unittests