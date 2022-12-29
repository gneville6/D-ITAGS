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
// Local
#include <grstapse/common/utilities/custom_json_conversions.hpp>
#include <grstapse/geometric_planning/graph/point/point_graph_environment.hpp>
#include <grstapse/geometric_planning/graph/point/point_graph_motion_planner.hpp>
#include <grstapse/geometric_planning/graph/point/point_graph_motion_planning_query_result.hpp>
#include <grstapse/geometric_planning/motion_planner_parameters_base.hpp>

namespace grstapse::unittests
{
    TEST(PointGraphMotionPlanner, simple)
    {
        std::ifstream in("data/geometric_planning/environments/point_graph.json");
        nlohmann::json j;
        in >> j;

        auto parameters = std::make_shared<const MotionPlannerParametersBase>(1.0f);
        auto graph      = j.get<std::shared_ptr<PointGraphEnvironment>>();
        PointGraphMotionPlanner mp(parameters, graph);

        auto initial_configuration = std::make_shared<const PointGraphConfiguration>(0, 0.0f, 0.0f);
        auto goal_configuration    = std::make_shared<const PointGraphConfiguration>(18, 4.0f, 4.0f);
        auto result                = std::dynamic_pointer_cast<const PointGraphMotionPlanningQueryResult>(
            mp.query(nullptr, initial_configuration, goal_configuration));
        ASSERT_EQ(result->status(), MotionPlannerQueryStatus::e_success);
        auto path = result->path();
        ASSERT_EQ(path.size(), 9);
    }
}  // namespace grstapse::unittests