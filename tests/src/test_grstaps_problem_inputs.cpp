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
#include <ompl/base/goals/GoalState.h>
// Project
#include <grstapse/core.hpp>
// Local
#include "mock_grstaps_problem_inputs.hpp"

namespace grstapse::unittests
{
    /**!
     * Tests GrstapsProblemInputs::loadMotionPlanners
     */
    TEST(GrstapProblemInputs, LoadMotionPlanners)
    {
        nlohmann::json j;
        {
            std::ifstream fin("data/grstaps_problem_inputs/motion_planners.json");
            fin >> j;
        }
        auto problem_inputs = std::make_shared<mocks::MockGrstapsProblemInputs>();
        problem_inputs->loadMotionPlanners(j);

        const std::vector<std::shared_ptr<EnvironmentBase>>& environments = problem_inputs->environments();
        ASSERT_EQ(environments.size(), 2);
        ASSERT_TRUE(std::dynamic_pointer_cast<PgmEnvironment>(environments[0]));
        ASSERT_TRUE(std::dynamic_pointer_cast<PgmEnvironment>(environments[1]));

        const std::vector<std::shared_ptr<MotionPlannerBase>>& motion_planners = problem_inputs->motionPlanners();
        ASSERT_EQ(motion_planners.size(), 2);
        ASSERT_EQ(std::dynamic_pointer_cast<OmplMotionPlanner>(motion_planners[0])->omplMotionPlannerType(),
                  OmplMotionPlannerType::e_prm);
        ASSERT_EQ(std::dynamic_pointer_cast<OmplMotionPlanner>(motion_planners[1])->omplMotionPlannerType(),
                  OmplMotionPlannerType::e_rrt);
    }

    /**!
     * Tests GrstapsProblemInputs::loadSpecies
     *
     * \note Relies on GrstapsProblemInputs::loadMotionPlanners working
     *
     * \todo(Andrew) Is there a way to test for correct MP
     */
    TEST(GrstapProblemInputs, LoadSpecies)
    {
        auto problem_inputs = std::make_shared<mocks::MockGrstapsProblemInputs>();

        {
            nlohmann::json j;
            std::ifstream fin("data/grstaps_problem_inputs/motion_planners.json");
            fin >> j;
            problem_inputs->loadMotionPlanners(j);
        }

        {
            std::ifstream fin("data/grstaps_problem_inputs/species.json");
            nlohmann::json j;
            fin >> j;
            auto [map, num_traits] = problem_inputs->loadSpecies(j);
            ASSERT_EQ(num_traits, 2);
        }

        const std::vector<std::shared_ptr<const Species>>& all_species = problem_inputs->multipleSpecies();
        ASSERT_EQ(all_species.size(), 2);

        {
            auto turtlebot3_burger = all_species[0];
            ASSERT_EQ(turtlebot3_burger->name(), "turtlebot3_burger");
            ASSERT_FLOAT_EQ(turtlebot3_burger->speed(), 0.2);
            ASSERT_FLOAT_EQ(turtlebot3_burger->boundingRadius(), 0.2);
            Eigen::VectorXf correct_traits(2);
            correct_traits << 0.2f, 15.f;
            ASSERT_EQ(turtlebot3_burger->traits(), correct_traits);
        }

        {
            auto turtlebot3_waffle = all_species[1];
            ASSERT_FLOAT_EQ(turtlebot3_waffle->speed(), 0.24);
            ASSERT_FLOAT_EQ(turtlebot3_waffle->boundingRadius(), 0.32);
            Eigen::VectorXf correct_traits(2);
            correct_traits << 0.24f, 30.f;
            ASSERT_EQ(turtlebot3_waffle->traits(), correct_traits);
        }
    }

    /**!
     * Tests GrstapsProblemInputs::loadRobots
     *
     * \note Relies on GrstapsProblemInputs::loadMotionPlanners and GrstapsProblemInputs::loadSpecies working
     */
    TEST(GrstapProblemInputs, LoadRobots)
    {
        auto problem_inputs = std::make_shared<mocks::MockGrstapsProblemInputs>();

        {
            nlohmann::json j;
            std::ifstream fin("data/grstaps_problem_inputs/motion_planners.json");
            fin >> j;
            problem_inputs->loadMotionPlanners(j);
        }

        {
            nlohmann::json species_j;
            {
                std::ifstream fin("data/grstaps_problem_inputs/species.json");
                fin >> species_j;
            }
            auto [map, num_traits] = problem_inputs->loadSpecies(species_j);

            nlohmann::json robots_j;
            {
                std::ifstream fin("data/grstaps_problem_inputs/robots.json");
                fin >> robots_j;
            }
            problem_inputs->loadRobots(map, num_traits, robots_j);
        }

        const std::vector<std::shared_ptr<const Robot>>& robots = problem_inputs->robots();
        ASSERT_EQ(robots.size(), 2);
        auto state_space = std::make_shared<ompl::base::SE2StateSpace>();

        {
            const std::shared_ptr<const Robot>& turtlebot3_burger = robots[0];
            ASSERT_EQ(turtlebot3_burger->name(), "turtlebot3_burger-1");

            auto correct_initial_configuration = std::make_shared<Se2StateOmplConfiguration>(0.0, 0.0, 0.0);
            ASSERT_TRUE(turtlebot3_burger->initialConfiguration()->isEqual(correct_initial_configuration));
            ASSERT_EQ(turtlebot3_burger->species()->name(), "turtlebot3_burger");
        }

        {
            const std::shared_ptr<const Robot>& turtlebot3_waffle = robots[1];
            ASSERT_EQ(turtlebot3_waffle->name(), "turtlebot3_waffle-1");
            auto correct_initial_configuration = std::make_shared<Se2StateOmplConfiguration>(10.0, 0.0, 0.0);
            ASSERT_TRUE(turtlebot3_waffle->initialConfiguration()->isEqual(correct_initial_configuration));
            ASSERT_EQ(turtlebot3_waffle->species()->name(), "turtlebot3_waffle");
        }
    }

    /**!
     * Tests GrstapsProblemInputs::createTasks
     *
     * \note depends on loadMotionPlanners working
     */
    TEST(GrstapProblemInputs, CreateTasks)
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
            std::ifstream fin("data/grstaps_problem_inputs/tasks.json");
            fin >> j;
        }

        // Create fake actions
        std::vector<std::shared_ptr<SasAction>> sas_actions;
        sas_actions.reserve(j.size());
        for(const auto& [key, val]: j.items())
        {
            sas_actions.push_back(std::make_shared<SasAction>(key, 0.0));
        }

        problem_inputs->createTasks(sas_actions, j);
        const std::vector<std::shared_ptr<const Task>>& tasks = problem_inputs->tasks();
        ASSERT_EQ(tasks.size(), 1);
        auto task = tasks[0];

        Eigen::VectorXf correct_desired_traits(2);
        correct_desired_traits << 0.0f, 10.0f;
        ASSERT_EQ(task->desiredTraits(), correct_desired_traits);

        auto state_space       = std::make_shared<ompl::base::SE2StateSpace>();
        auto space_information = std::make_shared<ompl::base::SpaceInformation>(state_space);
        {
            auto correct_initial_configuration = std::make_shared<Se2StateOmplConfiguration>(0.0, 0.0, 0.0);
            ASSERT_TRUE(task->initialConfiguration()->isEqual(correct_initial_configuration));
        }
        {
            auto correct_terminal_configuration = std::make_shared<Se2StateOmplConfiguration>(10.0, 0.0, 0.0);
            ASSERT_TRUE(task->terminalConfiguration()->isEqual(correct_terminal_configuration));
        }
    }

    TEST(GrstapProblemInputs, FullRun)
    {
        nlohmann::json j;
        std::ifstream fin("data/grstaps_problem_inputs/full_run.json");
        fin >> j;
        auto problem_inputs = j.get<std::shared_ptr<GrstapsProblemInputs>>();

        // Environments/Motion Planners
        {
            const std::vector<std::shared_ptr<EnvironmentBase>>& environments = problem_inputs->environments();
            ASSERT_EQ(environments.size(), 2);
            ASSERT_TRUE(std::dynamic_pointer_cast<PgmEnvironment>(environments[0]));
            ASSERT_TRUE(std::dynamic_pointer_cast<PgmEnvironment>(environments[0]));

            const std::vector<std::shared_ptr<MotionPlannerBase>>& motion_planners = problem_inputs->motionPlanners();
            ASSERT_EQ(motion_planners.size(), 2);
            ASSERT_EQ(std::dynamic_pointer_cast<OmplMotionPlanner>(motion_planners[0])->omplMotionPlannerType(),
                      OmplMotionPlannerType::e_prm);
            ASSERT_EQ(std::dynamic_pointer_cast<OmplMotionPlanner>(motion_planners[1])->omplMotionPlannerType(),
                      OmplMotionPlannerType::e_rrt);
        }

        // Robots
        {
            const std::vector<std::shared_ptr<const Robot>>& robots = problem_inputs->robots();
            ASSERT_EQ(robots.size(), 2);
            auto state_space = std::make_shared<ompl::base::SE2StateSpace>();

            {
                const std::shared_ptr<const Robot>& turtlebot3_burger = robots[0];
                ASSERT_EQ(turtlebot3_burger->name(), "turtlebot3_burger-1");

                auto correct_initial_configuration = std::make_shared<Se2StateOmplConfiguration>(0.0, 0.0, 0.0);
                ASSERT_TRUE(turtlebot3_burger->initialConfiguration()->isEqual(correct_initial_configuration));
                ASSERT_EQ(turtlebot3_burger->species()->name(), "turtlebot3_burger");
            }

            {
                const std::shared_ptr<const Robot>& turtlebot3_waffle = robots[1];
                ASSERT_EQ(turtlebot3_waffle->name(), "turtlebot3_waffle-1");
                auto correct_initial_configuration = std::make_shared<Se2StateOmplConfiguration>(10.0, 0.0, 0.0);
                ASSERT_TRUE(turtlebot3_waffle->initialConfiguration()->isEqual(correct_initial_configuration));
                ASSERT_EQ(turtlebot3_waffle->species()->name(), "turtlebot3_waffle");
            }
        }

        // Species
        {
            const std::vector<std::shared_ptr<const Species>>& all_species = problem_inputs->multipleSpecies();
            ASSERT_EQ(all_species.size(), 2);

            {
                auto turtlebot3_burger = all_species[0];
                ASSERT_EQ(turtlebot3_burger->name(), "turtlebot3_burger");
                ASSERT_FLOAT_EQ(turtlebot3_burger->speed(), 0.2);
                ASSERT_FLOAT_EQ(turtlebot3_burger->boundingRadius(), 0.2);
                Eigen::VectorXf correct_traits(2);
                correct_traits << 0.2f, 15.f;
                ASSERT_EQ(turtlebot3_burger->traits(), correct_traits);
            }

            {
                auto turtlebot3_waffle = all_species[1];
                ASSERT_FLOAT_EQ(turtlebot3_waffle->speed(), 0.24);
                ASSERT_FLOAT_EQ(turtlebot3_waffle->boundingRadius(), 0.32);
                Eigen::VectorXf correct_traits(2);
                correct_traits << 0.24f, 30.f;
                ASSERT_EQ(turtlebot3_waffle->traits(), correct_traits);
            }
        }
    }
}  // namespace grstapse::unittests