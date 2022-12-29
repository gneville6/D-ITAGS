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
#include <grstapse/geometric_planning/graph/point/sampled_point_graph_environment.hpp>

namespace grstapse::unittests
{
    TEST(SampledPointGraphEnvironment, Load)
    {
        std::ifstream in("data/geometric_planning/environments/sampled_point_graph.json");
        nlohmann::json j;
        in >> j;
        auto environment = j.get<std::shared_ptr<SampledPointGraphEnvironment>>();
        ASSERT_EQ(environment->numGraphs(), 3);
        ASSERT_EQ(environment->graph(0)->numVertices(), 19);
        ASSERT_EQ(environment->graph(0)->numEdges(), 22);
        ASSERT_EQ(environment->graph(1)->numVertices(), 19);
        ASSERT_EQ(environment->graph(1)->numEdges(), 8);
        ASSERT_EQ(environment->graph(2)->numVertices(), 19);
        ASSERT_EQ(environment->graph(2)->numEdges(), 10);
    }

    TEST(SampledPointGraphEnvironment, BaseLoad)
    {
        std::ifstream in("data/geometric_planning/environments/sampled_point_graph.json");
        nlohmann::json j;
        in >> j;
        std::shared_ptr<EnvironmentBase> environment_base = EnvironmentBase::deserializeFromJson(j);
        auto sampled_point_graph_environment =
            std::dynamic_pointer_cast<SampledPointGraphEnvironment>(environment_base);
        ASSERT_TRUE(sampled_point_graph_environment);
        ASSERT_EQ(sampled_point_graph_environment->numGraphs(), 3);
        ASSERT_EQ(sampled_point_graph_environment->graph(0)->numVertices(), 19);
        ASSERT_EQ(sampled_point_graph_environment->graph(0)->numEdges(), 22);
        ASSERT_EQ(sampled_point_graph_environment->graph(1)->numVertices(), 19);
        ASSERT_EQ(sampled_point_graph_environment->graph(1)->numEdges(), 8);
        ASSERT_EQ(sampled_point_graph_environment->graph(2)->numVertices(), 19);
        ASSERT_EQ(sampled_point_graph_environment->graph(2)->numEdges(), 10);
    }
}  // namespace grstapse::unittests