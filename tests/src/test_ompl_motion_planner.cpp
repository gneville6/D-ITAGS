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
#include <memory>
// External
#include <gtest/gtest.h>
#include <ompl/base/goals/GoalSpace.h>
#include <ompl/base/goals/GoalStates.h>
#include <ompl/base/spaces/SE2StateSpace.h>
// Project
#include <grstapse/common/utilities/custom_json_conversions.hpp>
#include <grstapse/geometric_planning/ompl/ompl_enums.hpp>
#include <grstapse/geometric_planning/ompl/ompl_motion_planner.hpp>
#include <grstapse/geometric_planning/ompl/ompl_motion_planner_parameters.hpp>
#include <grstapse/geometric_planning/ompl/ompl_motion_planning_query_result.hpp>
#include <grstapse/geometric_planning/ompl/se2_state_ompl_configuration.hpp>
#include <grstapse/geometric_planning/pgm_environment.hpp>

namespace grstapse::unittests
{
    TEST(MotionPlanner, MemoizationState)
    {
        auto parameters = std::make_shared<OmplMotionPlannerParameters>(
            /* timeout               = */ 1.0f,  // s
            /* simplify_path         = */ true,
            /* simplify_path_timeout = */ 1.0f,  // s
            /* connection_range      = */ 0.1f   // m
        );

        std::ifstream fin("data/geometric_planning/environments/pgm.json");
        nlohmann::json j;
        fin >> j;
        auto environment = j.get<std::shared_ptr<PgmEnvironment>>();

        auto motion_planner =
            std::make_shared<OmplMotionPlanner>(OmplMotionPlannerType::e_prm, parameters, environment);

        std::shared_ptr<const ompl::geometric::PathGeometric> path;
        auto species = std::make_shared<Species>("name", Eigen::VectorXf{}, 0.2f, 0.2f, motion_planner);

        // Run 1
        {
            auto initial_configuration = std::make_shared<Se2StateOmplConfiguration>(5.5, 0.0, 3.14159);
            auto goal_configuration    = std::make_shared<Se2StateOmplConfiguration>(-5.5, 0.0, 3.14159);

            std::shared_ptr<const MotionPlanningQueryResultBase> result =
                motion_planner->query(species, initial_configuration, goal_configuration);
            path = std::dynamic_pointer_cast<const OmplMotionPlanningQueryResult>(result)->path();
        }

        // Run 2
        {
            auto initial_configuration = std::make_shared<Se2StateOmplConfiguration>(5.5, 0.0, 3.14159);
            auto goal_configuration    = std::make_shared<Se2StateOmplConfiguration>(-5.5, 0.0, 3.14159);

            std::shared_ptr<const MotionPlanningQueryResultBase> result =
                motion_planner->query(species, initial_configuration, goal_configuration);
            ASSERT_EQ(path, std::dynamic_pointer_cast<const OmplMotionPlanningQueryResult>(result)->path());
        }
    }

    TEST(MotionPlanner, MemoizationStateDifferentRadius)
    {
        auto parameters = std::make_shared<OmplMotionPlannerParameters>(
            /* timeout               = */ 1.0f,  // s
            /* simplify_path         = */ true,
            /* simplify_path_timeout = */ 1.0f,  // s
            /* connection_range      = */ 0.1f   // m
        );

        std::ifstream fin("data/geometric_planning/environments/pgm.json");
        nlohmann::json j;
        fin >> j;
        auto environment = j.get<std::shared_ptr<PgmEnvironment>>();

        auto motion_planner =
            std::make_shared<OmplMotionPlanner>(OmplMotionPlannerType::e_prm, parameters, environment);

        std::shared_ptr<const ompl::geometric::PathGeometric> path;

        // Run 1
        {
            auto initial_configuration = std::make_shared<Se2StateOmplConfiguration>(5.5, 0.0, 3.14159);
            auto goal_configuration    = std::make_shared<Se2StateOmplConfiguration>(-5.5, 0.0, 3.14159);

            auto species = std::make_shared<Species>("name", Eigen::VectorXf{}, 0.2f, 0.2f, motion_planner);
            std::shared_ptr<const MotionPlanningQueryResultBase> result =
                motion_planner->query(species, initial_configuration, goal_configuration);
            path = std::dynamic_pointer_cast<const OmplMotionPlanningQueryResult>(result)->path();
        }

        // Run 2
        {
            auto initial_configuration = std::make_shared<Se2StateOmplConfiguration>(5.5, 0.0, 3.14159);
            auto goal_configuration    = std::make_shared<Se2StateOmplConfiguration>(-5.5, 0.0, 3.14159);

            auto species = std::make_shared<Species>("name", Eigen::VectorXf{}, 0.3f, 0.2f, motion_planner);
            std::shared_ptr<const MotionPlanningQueryResultBase> result =
                motion_planner->query(species, initial_configuration, goal_configuration);
            ASSERT_NE(path, std::dynamic_pointer_cast<const OmplMotionPlanningQueryResult>(result)->path());
        }
    }

    TEST(MotionPlanner, MemoizationStateDifferentStartState)
    {
        auto parameters = std::make_shared<OmplMotionPlannerParameters>(
            /* timeout               = */ 1.0f,  // s
            /* simplify_path         = */ true,
            /* simplify_path_timeout = */ 1.0f,  // s
            /* connection_range      = */ 0.1f   // m
        );

        std::ifstream fin("data/geometric_planning/environments/pgm.json");
        nlohmann::json j;
        fin >> j;
        auto environment = j.get<std::shared_ptr<PgmEnvironment>>();

        auto motion_planner =
            std::make_shared<OmplMotionPlanner>(OmplMotionPlannerType::e_prm, parameters, environment);

        std::shared_ptr<const ompl::geometric::PathGeometric> path;
        auto species = std::make_shared<Species>("name", Eigen::VectorXf{}, 0.2f, 0.2f, motion_planner);

        // Run 1
        {
            auto initial_configuration = std::make_shared<Se2StateOmplConfiguration>(5.5, 0.0, 3.14159);
            auto goal_configuration    = std::make_shared<Se2StateOmplConfiguration>(-5.5, 0.0, 3.14159);

            std::shared_ptr<const MotionPlanningQueryResultBase> result =
                motion_planner->query(species, initial_configuration, goal_configuration);
            path = std::dynamic_pointer_cast<const OmplMotionPlanningQueryResult>(result)->path();
        }

        // Run 2
        {
            auto initial_configuration = std::make_shared<Se2StateOmplConfiguration>(4.5, 0.0, 3.14159);
            auto goal_configuration    = std::make_shared<Se2StateOmplConfiguration>(-5.5, 0.0, 3.14159);

            std::shared_ptr<const MotionPlanningQueryResultBase> result =
                motion_planner->query(species, initial_configuration, goal_configuration);
            ASSERT_NE(path, std::dynamic_pointer_cast<const OmplMotionPlanningQueryResult>(result)->path());
        }
    }

    TEST(MotionPlanner, MemoizationStateDifferentState)
    {
        auto parameters = std::make_shared<OmplMotionPlannerParameters>(
            /* timeout               = */ 1.0f,  // s
            /* simplify_path         = */ true,
            /* simplify_path_timeout = */ 1.0f,  // s
            /* connection_range      = */ 0.1f   // m
        );

        std::ifstream fin("data/geometric_planning/environments/pgm.json");
        nlohmann::json j;
        fin >> j;
        auto environment = j.get<std::shared_ptr<PgmEnvironment>>();

        auto motion_planner =
            std::make_shared<OmplMotionPlanner>(OmplMotionPlannerType::e_prm, parameters, environment);

        std::shared_ptr<const ompl::geometric::PathGeometric> path;
        auto species = std::make_shared<Species>("name", Eigen::VectorXf{}, 0.2f, 0.2f, motion_planner);

        // Run 1
        {
            auto initial_configuration = std::make_shared<Se2StateOmplConfiguration>(5.5, 0.0, 3.14159);
            auto goal_configuration    = std::make_shared<Se2StateOmplConfiguration>(-5.5, 0.0, 3.14159);

            std::shared_ptr<const MotionPlanningQueryResultBase> result =
                motion_planner->query(species, initial_configuration, goal_configuration);
            path = std::dynamic_pointer_cast<const OmplMotionPlanningQueryResult>(result)->path();
        }

        // Run 2
        {
            auto initial_configuration = std::make_shared<Se2StateOmplConfiguration>(5.5, 0.0, 3.14159);
            auto goal_configuration    = std::make_shared<Se2StateOmplConfiguration>(-4.5, 0.0, 3.14159);

            std::shared_ptr<const MotionPlanningQueryResultBase> result =
                motion_planner->query(species, initial_configuration, goal_configuration);
            ASSERT_NE(path, std::dynamic_pointer_cast<const OmplMotionPlanningQueryResult>(result)->path());
        }
    }

    TEST(MotionPlanner, PrmStar)
    {
        auto parameters = std::make_shared<OmplMotionPlannerParameters>(
            /* timeout               = */ 1.0f,  // s
            /* simplify_path         = */ true,
            /* simplify_path_timeout = */ 1.0f,  // s
            /* connection_range      = */ 0.1f   // m
        );

        std::ifstream fin("data/geometric_planning/environments/pgm.json");
        nlohmann::json j;
        fin >> j;
        auto environment = j.get<std::shared_ptr<PgmEnvironment>>();

        auto motion_planner =
            std::make_shared<OmplMotionPlanner>(OmplMotionPlannerType::e_prm_star, parameters, environment);

        auto initial_configuration = std::make_shared<Se2StateOmplConfiguration>(5.5, 0.0, 3.14159);
        auto goal_configuration    = std::make_shared<Se2StateOmplConfiguration>(-5.5, 0.0, 3.14159);

        auto species = std::make_shared<Species>("name", Eigen::VectorXf{}, 0.2f, 0.2f, motion_planner);
        std::shared_ptr<const MotionPlanningQueryResultBase> result =
            motion_planner->query(species, initial_configuration, goal_configuration);

        ASSERT_EQ(result->status(), MotionPlannerQueryStatus::e_success);

        const auto& path = std::dynamic_pointer_cast<const OmplMotionPlanningQueryResult>(result)->path();
        ASSERT_EQ(path->getStateCount(), 2);
        const auto* state_0 = dynamic_cast<const ompl::base::SE2StateSpace::StateType*>(path->getState(0));
        ASSERT_FLOAT_EQ(state_0->getX(), 5.5);
        ASSERT_FLOAT_EQ(state_0->getY(), 0.0);
        ASSERT_FLOAT_EQ(state_0->getYaw(), 3.14159);
        const auto* state_1 = dynamic_cast<const ompl::base::SE2StateSpace::StateType*>(path->getState(1));
        ASSERT_FLOAT_EQ(state_1->getX(), -5.5);
        ASSERT_FLOAT_EQ(state_1->getY(), 0.0);
        ASSERT_FLOAT_EQ(state_1->getYaw(), 3.14159);
    }

    TEST(MotionPlanner, LazyPrm)
    {
        auto parameters = std::make_shared<OmplMotionPlannerParameters>(
            /* timeout               = */ 1.0f,  // s
            /* simplify_path         = */ true,
            /* simplify_path_timeout = */ 1.0f,  // s
            /* connection_range      = */ 0.1f   // m
        );

        std::ifstream fin("data/geometric_planning/environments/pgm.json");
        nlohmann::json j;
        fin >> j;
        auto environment = j.get<std::shared_ptr<PgmEnvironment>>();

        auto motion_planner =
            std::make_shared<OmplMotionPlanner>(OmplMotionPlannerType::e_lazy_prm, parameters, environment);

        auto initial_configuration = std::make_shared<Se2StateOmplConfiguration>(5.5, 0.0, 3.14159);
        auto goal_configuration    = std::make_shared<Se2StateOmplConfiguration>(-5.5, 0.0, 3.14159);

        auto species = std::make_shared<Species>("name", Eigen::VectorXf{}, 0.2f, 0.2f, motion_planner);
        std::shared_ptr<const MotionPlanningQueryResultBase> result =
            motion_planner->query(species, initial_configuration, goal_configuration);

        ASSERT_EQ(result->status(), MotionPlannerQueryStatus::e_success);

        const auto& path = std::dynamic_pointer_cast<const OmplMotionPlanningQueryResult>(result)->path();
        ASSERT_EQ(path->getStateCount(), 2);
        const auto* state_0 = dynamic_cast<const ompl::base::SE2StateSpace::StateType*>(path->getState(0));
        ASSERT_FLOAT_EQ(state_0->getX(), 5.5);
        ASSERT_FLOAT_EQ(state_0->getY(), 0.0);
        ASSERT_FLOAT_EQ(state_0->getYaw(), 3.14159);
        const auto* state_1 = dynamic_cast<const ompl::base::SE2StateSpace::StateType*>(path->getState(1));
        ASSERT_FLOAT_EQ(state_1->getX(), -5.5);
        ASSERT_FLOAT_EQ(state_1->getY(), 0.0);
        ASSERT_FLOAT_EQ(state_1->getYaw(), 3.14159);
    }

    TEST(MotionPlanner, LazyPrmStar)
    {
        auto parameters = std::make_shared<OmplMotionPlannerParameters>(
            /* timeout               = */ 1.0f,  // s
            /* simplify_path         = */ true,
            /* simplify_path_timeout = */ 1.0f,  // s
            /* connection_range      = */ 0.1f   // m
        );

        std::ifstream fin("data/geometric_planning/environments/pgm.json");
        nlohmann::json j;
        fin >> j;
        auto environment = j.get<std::shared_ptr<PgmEnvironment>>();

        auto motion_planner =
            std::make_shared<OmplMotionPlanner>(OmplMotionPlannerType::e_lazy_prm_star, parameters, environment);

        auto initial_configuration = std::make_shared<Se2StateOmplConfiguration>(5.5, 0.0, 3.14159);
        auto goal_configuration    = std::make_shared<Se2StateOmplConfiguration>(-5.5, 0.0, 3.14159);

        auto species = std::make_shared<Species>("name", Eigen::VectorXf{}, 0.2f, 0.2f, motion_planner);
        std::shared_ptr<const MotionPlanningQueryResultBase> result =
            motion_planner->query(species, initial_configuration, goal_configuration);

        ASSERT_EQ(result->status(), MotionPlannerQueryStatus::e_success);

        const auto& path = std::dynamic_pointer_cast<const OmplMotionPlanningQueryResult>(result)->path();
        ASSERT_EQ(path->getStateCount(), 2);
        const auto* state_0 = dynamic_cast<const ompl::base::SE2StateSpace::StateType*>(path->getState(0));
        ASSERT_FLOAT_EQ(state_0->getX(), 5.5);
        ASSERT_FLOAT_EQ(state_0->getY(), 0.0);
        ASSERT_FLOAT_EQ(state_0->getYaw(), 3.14159);
        const auto* state_1 = dynamic_cast<const ompl::base::SE2StateSpace::StateType*>(path->getState(1));
        ASSERT_FLOAT_EQ(state_1->getX(), -5.5);
        ASSERT_FLOAT_EQ(state_1->getY(), 0.0);
        ASSERT_FLOAT_EQ(state_1->getYaw(), 3.14159);
    }

    TEST(MotionPlanner, Rrt)
    {
        auto parameters = std::make_shared<OmplMotionPlannerParameters>(
            /* timeout               = */ 1.0f,  // s
            /* simplify_path         = */ true,
            /* simplify_path_timeout = */ 1.0f,  // s
            /* connection_range      = */ 0.1f   // m
        );

        std::ifstream fin("data/geometric_planning/environments/pgm.json");
        nlohmann::json j;
        fin >> j;
        auto environment = j.get<std::shared_ptr<PgmEnvironment>>();

        auto motion_planner =
            std::make_shared<OmplMotionPlanner>(OmplMotionPlannerType::e_rrt, parameters, environment);

        auto initial_configuration = std::make_shared<Se2StateOmplConfiguration>(5.5, 0.0, 3.14159);
        auto goal_configuration    = std::make_shared<Se2StateOmplConfiguration>(-5.5, 0.0, 3.14159);

        auto species = std::make_shared<Species>("name", Eigen::VectorXf{}, 0.2f, 0.2f, motion_planner);
        std::shared_ptr<const MotionPlanningQueryResultBase> result =
            motion_planner->query(species, initial_configuration, goal_configuration);

        ASSERT_EQ(result->status(), MotionPlannerQueryStatus::e_success);

        const auto& path = std::dynamic_pointer_cast<const OmplMotionPlanningQueryResult>(result)->path();
        ASSERT_EQ(path->getStateCount(), 2);
        const auto* state_0 = dynamic_cast<const ompl::base::SE2StateSpace::StateType*>(path->getState(0));
        ASSERT_FLOAT_EQ(state_0->getX(), 5.5);
        ASSERT_FLOAT_EQ(state_0->getY(), 0.0);
        ASSERT_FLOAT_EQ(state_0->getYaw(), 3.14159);
        const auto* state_1 = dynamic_cast<const ompl::base::SE2StateSpace::StateType*>(path->getState(1));
        ASSERT_FLOAT_EQ(state_1->getX(), -5.5);
        ASSERT_FLOAT_EQ(state_1->getY(), 0.0);
        ASSERT_FLOAT_EQ(state_1->getYaw(), 3.14159);
    }

    TEST(MotionPlanner, RrtStar)
    {
        auto parameters = std::make_shared<OmplMotionPlannerParameters>(
            /* timeout               = */ 1.0f,  // s
            /* simplify_path         = */ true,
            /* simplify_path_timeout = */ 1.0f,  // s
            /* connection_range      = */ 0.1f   // m
        );

        std::ifstream fin("data/geometric_planning/environments/pgm.json");
        nlohmann::json j;
        fin >> j;
        auto environment = j.get<std::shared_ptr<PgmEnvironment>>();

        auto motion_planner =
            std::make_shared<OmplMotionPlanner>(OmplMotionPlannerType::e_rrt_star, parameters, environment);

        auto initial_configuration = std::make_shared<Se2StateOmplConfiguration>(5.5, 0.0, 3.14159);
        auto goal_configuration    = std::make_shared<Se2StateOmplConfiguration>(-5.5, 0.0, 3.14159);

        auto species = std::make_shared<Species>("name", Eigen::VectorXf{}, 0.2f, 0.2f, motion_planner);
        std::shared_ptr<const MotionPlanningQueryResultBase> result =
            motion_planner->query(species, initial_configuration, goal_configuration);

        ASSERT_EQ(result->status(), MotionPlannerQueryStatus::e_success);

        const auto& path = std::dynamic_pointer_cast<const OmplMotionPlanningQueryResult>(result)->path();
        ASSERT_EQ(path->getStateCount(), 2);
        const auto* state_0 = dynamic_cast<const ompl::base::SE2StateSpace::StateType*>(path->getState(0));
        ASSERT_FLOAT_EQ(state_0->getX(), 5.5);
        ASSERT_FLOAT_EQ(state_0->getY(), 0.0);
        ASSERT_FLOAT_EQ(state_0->getYaw(), 3.14159);
        const auto* state_1 = dynamic_cast<const ompl::base::SE2StateSpace::StateType*>(path->getState(1));
        ASSERT_FLOAT_EQ(state_1->getX(), -5.5);
        ASSERT_FLOAT_EQ(state_1->getY(), 0.0);
        ASSERT_FLOAT_EQ(state_1->getYaw(), 3.14159);
    }

    TEST(MotionPlanner, RrtConnect)
    {
        auto parameters = std::make_shared<OmplMotionPlannerParameters>(
            /* timeout               = */ 1.0f,  // s
            /* simplify_path         = */ true,
            /* simplify_path_timeout = */ 1.0f,  // s
            /* connection_range      = */ 0.1f   // m
        );

        std::ifstream fin("data/geometric_planning/environments/pgm.json");
        nlohmann::json j;
        fin >> j;
        auto environment = j.get<std::shared_ptr<PgmEnvironment>>();

        auto motion_planner =
            std::make_shared<OmplMotionPlanner>(OmplMotionPlannerType::e_rrt_connect, parameters, environment);

        auto initial_configuration = std::make_shared<Se2StateOmplConfiguration>(5.5, 0.0, 3.14159);
        auto goal_configuration    = std::make_shared<Se2StateOmplConfiguration>(-5.5, 0.0, 3.14159);

        auto species = std::make_shared<Species>("name", Eigen::VectorXf{}, 0.2f, 0.2f, motion_planner);
        std::shared_ptr<const MotionPlanningQueryResultBase> result =
            motion_planner->query(species, initial_configuration, goal_configuration);

        ASSERT_EQ(result->status(), MotionPlannerQueryStatus::e_success);

        const auto& path = std::dynamic_pointer_cast<const OmplMotionPlanningQueryResult>(result)->path();
        ASSERT_EQ(path->getStateCount(), 2);
        const auto* state_0 = dynamic_cast<const ompl::base::SE2StateSpace::StateType*>(path->getState(0));
        ASSERT_FLOAT_EQ(state_0->getX(), 5.5);
        ASSERT_FLOAT_EQ(state_0->getY(), 0.0);
        ASSERT_FLOAT_EQ(state_0->getYaw(), 3.14159);
        const auto* state_1 = dynamic_cast<const ompl::base::SE2StateSpace::StateType*>(path->getState(1));
        ASSERT_FLOAT_EQ(state_1->getX(), -5.5);
        ASSERT_FLOAT_EQ(state_1->getY(), 0.0);
        ASSERT_FLOAT_EQ(state_1->getYaw(), 3.14159);
    }

    TEST(MotionPlanner, ParallelRrt)
    {
        auto parameters = std::make_shared<OmplMotionPlannerParameters>(
            /* timeout               = */ 1.0f,  // s
            /* simplify_path         = */ true,
            /* simplify_path_timeout = */ 1.0f,  // s
            /* connection_range      = */ 0.1f   // m
        );

        std::ifstream fin("data/geometric_planning/environments/pgm.json");
        nlohmann::json j;
        fin >> j;
        auto environment = j.get<std::shared_ptr<PgmEnvironment>>();

        auto motion_planner =
            std::make_shared<OmplMotionPlanner>(OmplMotionPlannerType::e_parallel_rrt, parameters, environment);

        auto initial_configuration = std::make_shared<Se2StateOmplConfiguration>(5.5, 0.0, 3.14159);
        auto goal_configuration    = std::make_shared<Se2StateOmplConfiguration>(-5.5, 0.0, 3.14159);

        auto species = std::make_shared<Species>("name", Eigen::VectorXf{}, 0.2f, 0.2f, motion_planner);
        std::shared_ptr<const MotionPlanningQueryResultBase> result =
            motion_planner->query(species, initial_configuration, goal_configuration);

        ASSERT_EQ(result->status(), MotionPlannerQueryStatus::e_success);

        const auto& path = std::dynamic_pointer_cast<const OmplMotionPlanningQueryResult>(result)->path();
        ASSERT_EQ(path->getStateCount(), 2);
        const auto* state_0 = dynamic_cast<const ompl::base::SE2StateSpace::StateType*>(path->getState(0));
        ASSERT_FLOAT_EQ(state_0->getX(), 5.5);
        ASSERT_FLOAT_EQ(state_0->getY(), 0.0);
        ASSERT_FLOAT_EQ(state_0->getYaw(), 3.14159);
        const auto* state_1 = dynamic_cast<const ompl::base::SE2StateSpace::StateType*>(path->getState(1));
        ASSERT_FLOAT_EQ(state_1->getX(), -5.5);
        ASSERT_FLOAT_EQ(state_1->getY(), 0.0);
        ASSERT_FLOAT_EQ(state_1->getYaw(), 3.14159);
    }

    TEST(MotionPlanner, LazyRrt)
    {
        auto parameters = std::make_shared<OmplMotionPlannerParameters>(
            /* timeout               = */ 1.0f,  // s
            /* simplify_path         = */ true,
            /* simplify_path_timeout = */ 1.0f,  // s
            /* connection_range      = */ 0.1f   // m
        );

        std::ifstream fin("data/geometric_planning/environments/pgm.json");
        nlohmann::json j;
        fin >> j;
        auto environment = j.get<std::shared_ptr<PgmEnvironment>>();

        auto motion_planner =
            std::make_shared<OmplMotionPlanner>(OmplMotionPlannerType::e_lazy_rrt, parameters, environment);

        auto initial_configuration = std::make_shared<Se2StateOmplConfiguration>(5.5, 0.0, 3.14159);
        auto goal_configuration    = std::make_shared<Se2StateOmplConfiguration>(-5.5, 0.0, 3.14159);

        auto species = std::make_shared<Species>("name", Eigen::VectorXf{}, 0.2f, 0.2f, motion_planner);
        std::shared_ptr<const MotionPlanningQueryResultBase> result =
            motion_planner->query(species, initial_configuration, goal_configuration);

        ASSERT_EQ(result->status(), MotionPlannerQueryStatus::e_success);

        const auto& path = std::dynamic_pointer_cast<const OmplMotionPlanningQueryResult>(result)->path();
        ASSERT_EQ(path->getStateCount(), 2);
        const auto* state_0 = dynamic_cast<const ompl::base::SE2StateSpace::StateType*>(path->getState(0));
        ASSERT_FLOAT_EQ(state_0->getX(), 5.5);
        ASSERT_FLOAT_EQ(state_0->getY(), 0.0);
        ASSERT_FLOAT_EQ(state_0->getYaw(), 3.14159);
        const auto* state_1 = dynamic_cast<const ompl::base::SE2StateSpace::StateType*>(path->getState(1));
        ASSERT_FLOAT_EQ(state_1->getX(), -5.5);
        ASSERT_FLOAT_EQ(state_1->getY(), 0.0);
        ASSERT_FLOAT_EQ(state_1->getYaw(), 3.14159);
    }
}  // namespace grstapse::unittests