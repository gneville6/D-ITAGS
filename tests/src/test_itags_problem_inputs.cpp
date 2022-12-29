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
#include <ompl/base/goals/GoalState.h>
#include <ompl/base/spaces/SE2StateSpace.h>
// Project
#include <grstapse/common/utilities/custom_json_conversions.hpp>
#include <grstapse/geometric_planning/ompl/se2_state_ompl_configuration.hpp>
#include <grstapse/task.hpp>
// Local
#include "mock_grstaps_problem_inputs.hpp"
#include "mock_itags_problem_inputs.hpp"

namespace grstapse::unittests
{
    TEST(ItagsProblemInputs, LoadTasks)
    {
        auto gp = std::make_shared<mocks::MockGrstapsProblemInputs>();
        {
            nlohmann::json j;
            std::ifstream fin("data/grstaps_problem_inputs/motion_planners.json");
            fin >> j;
            gp->loadMotionPlanners(j);
        }

        nlohmann::json j;
        {
            std::ifstream fin("data/task_allocation/itags_problem_inputs/load_tasks.json");
            fin >> j;
        }
        mocks::MockItagsProblemInputs p;
        std::vector<std::shared_ptr<const Task>> tasks = p.loadTasks(j, gp);
        ASSERT_EQ(tasks.size(), 1);
        {
            ASSERT_EQ(tasks[0]->name(), "move_box_a");

            Eigen::VectorXf correct_desired_traits(2);
            correct_desired_traits << 0.0f, 10.0f;
            ASSERT_EQ(tasks[0]->desiredTraits(), correct_desired_traits);

            auto state_space       = std::make_shared<ompl::base::SE2StateSpace>();
            auto space_information = std::make_shared<ompl::base::SpaceInformation>(state_space);
            {
                auto correct_initial_configuration = std::make_shared<Se2StateOmplConfiguration>(0.0f, 0.0f, 0.0f);
                ASSERT_TRUE(tasks[0]->initialConfiguration()->isEqual(correct_initial_configuration));
            }
            {
                auto correct_terminal_configuration = std::make_shared<Se2StateOmplConfiguration>(10.0f, 0.0f, 0.0f);
                ASSERT_TRUE(tasks[0]->terminalConfiguration()->isEqual(correct_terminal_configuration));
            }
        }
    }
#ifndef NO_MILP

    TEST(ItagsProblemInputs, Load)
    {
        std::ifstream fin("data/task_allocation/itags_problem_inputs/full_run.json");
        nlohmann::json j;
        fin >> j;

        auto p = j.get<std::shared_ptr<ItagsProblemInputs>>();

        // todo(Andrew): Add asserts
    }
#endif
}  // namespace grstapse::unittests