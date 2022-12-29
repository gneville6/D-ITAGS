/*
 * Graphically Recursive Simultaneous Task Allocation, Planning,
 * Scheduling, and Execution
 *
 * Copyright (C) 2020-2021
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
#include <grstapse/core.hpp>
// Local
#include "mock_grstaps_problem_inputs.hpp"

namespace grstapse::unittests
{
    /**!
     * \note Depends on GrstapsProblemInputs::loadMotionPlanners
     */
    TEST(Species, from_json)
    {
        auto problem_inputs = std::make_shared<mocks::MockGrstapsProblemInputs>();

        // Note: needed to setup permissible configuration spaces
        {
            nlohmann::json j;
            std::ifstream fin("data/grstaps_problem_inputs/motion_planners.json");
            fin >> j;
            problem_inputs->loadMotionPlanners(j);
        }

        nlohmann::json j;
        {
            std::ifstream fin("data/species/turtlebot3_burger.json");
            fin >> j;
        }

        auto turtlebot3_burger = Species::loadJson(j, problem_inputs->motionPlanners());
        ASSERT_EQ(turtlebot3_burger->name(), "turtlebot3_burger");
        ASSERT_FLOAT_EQ(turtlebot3_burger->speed(), 0.2);
        ASSERT_FLOAT_EQ(turtlebot3_burger->boundingRadius(), 0.2);
        Eigen::VectorXf correct_traits(2, 1);
        correct_traits << 0.2f, 15.f;
        ASSERT_EQ(turtlebot3_burger->traits(), correct_traits);
    }
}  // namespace grstapse::unittests