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
#include <grstapse/geometric_planning/graph/point/point_graph_configuration.hpp>
#include <grstapse/geometric_planning/graph/point/point_graph_environment.hpp>
#include <grstapse/geometric_planning/graph/point/point_graph_motion_planning_query_result.hpp>
#include <grstapse/geometric_planning/graph/point/sampled_point_graph_motion_planner.hpp>
#include <grstapse/geometric_planning/motion_planner_parameters_base.hpp>

namespace grstapse::unittests
{
    TEST(SampledPointGraphMotionPlanner, Load)
    {
        auto parameters = std::make_shared<MotionPlannerParametersBase>(
            /* timeout               = */ 1.0f  // s
        );

        std::ifstream fin("data/geometric_planning/environments/sampled_point_graph.json");
        nlohmann::json j;
        fin >> j;
        auto environment = j.get<std::shared_ptr<SampledPointGraphEnvironment>>();

        auto motion_planner = std::make_shared<SampledPointGraphMotionPlanner>(parameters, environment);

        auto ic = std::make_shared<PointGraphConfiguration>(0, 0.0f, 0.0f);
        auto gc = std::make_shared<PointGraphConfiguration>(18, 4.0f, 4.0f);

        {
            auto result = motion_planner->query(0, nullptr, ic, gc);
            auto path   = std::dynamic_pointer_cast<const PointGraphMotionPlanningQueryResult>(result)->path();
            ASSERT_EQ(path.size(), 9);
        }
        {
            auto result = motion_planner->query(1, nullptr, ic, gc);
            auto path   = std::dynamic_pointer_cast<const PointGraphMotionPlanningQueryResult>(result)->path();
            ASSERT_EQ(path.size(), 9);
        }
        {
            auto result = motion_planner->query(2, nullptr, ic, gc);
            auto path   = std::dynamic_pointer_cast<const PointGraphMotionPlanningQueryResult>(result)->path();
            ASSERT_EQ(path.size(), 11);
        }
    }
}  // namespace grstapse::unittests