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
#include <memory>
// External
#include <gtest/gtest.h>
#include <robin_hood/robin_hood.hpp>
// Project
#include <grstapse/geometric_planning/grid/grid_cell.hpp>
#include <grstapse/geometric_planning/grid/grid_map.hpp>
#include <grstapse/geometric_planning/grid/grid_search.hpp>

namespace grstapse::unittests
{
    void assertGridCell(const std::shared_ptr<const GridCellNode>& node, const GridCell& cell)
    {
        ASSERT_EQ(node->x(), cell.x());
        ASSERT_EQ(node->y(), cell.y());
    }

    void assertGridCell(const std::shared_ptr<const GridCellNode>& node, std::shared_ptr<const GridCell> cell)
    {
        assertGridCell(node, *cell);
    }

    void assertRoute(const std::shared_ptr<const GridCellNode>& goal, const std::vector<GridCell>& expected_route)
    {
        std::vector<std::shared_ptr<const GridCellNode>> route = {goal};
        std::shared_ptr<const GridCellNode> parent             = goal->parent();
        while(parent != nullptr)
        {
            route.push_back(parent);
            parent = parent->parent();
        }
        ASSERT_EQ(route.size(), expected_route.size());
        for(unsigned int i = 0, end = route.size(); i < end; ++i)
        {
            assertGridCell(route[i], expected_route[end - i - 1]);
        }
    }

    TEST(AStar, Map3x3)
    {
        auto parameters = std::make_shared<const BestFirstSearchParameters>(false, 0.0, "a_star", false, false);

        robin_hood::unordered_set<GridCell> obstacles = {GridCell(1, 1), GridCell(2, 2)};

        auto map     = std::make_shared<const GridMap>(3, 3, obstacles);
        auto initial = std::make_shared<const GridCell>(0, 0);
        auto goal    = std::make_shared<const GridCell>(1, 2);

        GridSearch grid_search(parameters, map, initial, goal);
        SearchResults<GridCellNode, SearchStatisticsCommon> solution = grid_search.search();
        ASSERT_TRUE(solution.foundGoal());

        std::shared_ptr<GridCellNode> goal_node = solution.goal();
        assertGridCell(goal_node, goal);
        assertRoute(goal_node, {{0, 0}, {0, 1}, {0, 2}, {1, 2}});
    }
}  // namespace grstapse::unittests