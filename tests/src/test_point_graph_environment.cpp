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
#include <grstapse/common/utilities/custom_json_conversions.hpp>
#include <grstapse/geometric_planning/graph/point/point_graph_environment.hpp>

namespace grstapse::unittests
{
    TEST(PointGraphEnvironment, Load)
    {
        std::ifstream in("data/geometric_planning/environments/point_graph.json");
        nlohmann::json j;
        in >> j;
        auto environment = j.get<std::shared_ptr<PointGraphEnvironment>>();
        ASSERT_EQ(environment->numVertices(), 19);
        ASSERT_EQ(environment->numEdges(), 22);
    }

    TEST(PointGraphEnvironment, BaseLoad)
    {
        std::ifstream in("data/geometric_planning/environments/point_graph.json");
        nlohmann::json j;
        in >> j;
        std::shared_ptr<EnvironmentBase> environment = EnvironmentBase::deserializeFromJson(j);
        auto point_graph_environment                 = std::dynamic_pointer_cast<PointGraphEnvironment>(environment);
        ASSERT_TRUE(point_graph_environment);
        ASSERT_EQ(point_graph_environment->numVertices(), 19);
        ASSERT_EQ(point_graph_environment->numEdges(), 22);
    }
}  // namespace grstapse::unittests