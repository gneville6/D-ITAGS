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
#include <grstapse/geometric_planning/ompl/se2_state_ompl_configuration.hpp>
#include <grstapse/geometric_planning/ompl/se3_state_ompl_configuration.hpp>

namespace grstapse::unittests
{
    TEST(ConfigurationBase, LoadSe2State)
    {
        std::ifstream in("data/geometric_planning/ompl/se2_state.json");
        nlohmann::json j;
        in >> j;

        auto configuration = Se2StateOmplConfiguration::deserializeFromJson(j);
        ASSERT_EQ(configuration->configurationType(), ConfigurationType::e_ompl);
        ASSERT_EQ(configuration->goalType(), OmplGoalType::e_state);
        ASSERT_EQ(configuration->stateSpaceType(), OmplStateSpaceType::e_se2);
        ASSERT_FLOAT_EQ(configuration->x(), 1.0f);
        ASSERT_FLOAT_EQ(configuration->y(), 0.0f);
        ASSERT_FLOAT_EQ(configuration->yaw(), 0.0f);
    }

    TEST(ConfigurationBase, LoadSe2OmplConfiguration_Se2State)
    {
        std::ifstream in("data/geometric_planning/ompl/se2_state.json");
        nlohmann::json j;
        in >> j;

        auto configuration     = Se2OmplConfiguration::deserializeFromJson(j);
        auto se2_configuration = std::dynamic_pointer_cast<const Se2StateOmplConfiguration>(configuration);
        ASSERT_TRUE(se2_configuration);
        ASSERT_EQ(se2_configuration->configurationType(), ConfigurationType::e_ompl);
        ASSERT_EQ(se2_configuration->goalType(), OmplGoalType::e_state);
        ASSERT_EQ(se2_configuration->stateSpaceType(), OmplStateSpaceType::e_se2);
        ASSERT_FLOAT_EQ(se2_configuration->x(), 1.0f);
        ASSERT_FLOAT_EQ(se2_configuration->y(), 0.0f);
        ASSERT_FLOAT_EQ(se2_configuration->yaw(), 0.0f);
    }

    TEST(ConfigurationBase, LoadOmplConfiguration_Se2State)
    {
        std::ifstream in("data/geometric_planning/ompl/se2_state.json");
        nlohmann::json j;
        in >> j;

        auto configuration     = OmplConfiguration::deserializeFromJson(j);
        auto se2_configuration = std::dynamic_pointer_cast<const Se2StateOmplConfiguration>(configuration);
        ASSERT_TRUE(se2_configuration);
        ASSERT_EQ(se2_configuration->configurationType(), ConfigurationType::e_ompl);
        ASSERT_EQ(se2_configuration->goalType(), OmplGoalType::e_state);
        ASSERT_EQ(se2_configuration->stateSpaceType(), OmplStateSpaceType::e_se2);
        ASSERT_FLOAT_EQ(se2_configuration->x(), 1.0f);
        ASSERT_FLOAT_EQ(se2_configuration->y(), 0.0f);
        ASSERT_FLOAT_EQ(se2_configuration->yaw(), 0.0f);
    }

    TEST(ConfigurationBase, LoadConfigurationBase_Se2State)
    {
        std::ifstream in("data/geometric_planning/ompl/se2_state.json");
        nlohmann::json j;
        in >> j;

        auto configuration     = ConfigurationBase::deserializeFromJson(j);
        auto se2_configuration = std::dynamic_pointer_cast<const Se2StateOmplConfiguration>(configuration);
        ASSERT_TRUE(se2_configuration);
        ASSERT_EQ(se2_configuration->configurationType(), ConfigurationType::e_ompl);
        ASSERT_EQ(se2_configuration->goalType(), OmplGoalType::e_state);
        ASSERT_EQ(se2_configuration->stateSpaceType(), OmplStateSpaceType::e_se2);
        ASSERT_FLOAT_EQ(se2_configuration->x(), 1.0f);
        ASSERT_FLOAT_EQ(se2_configuration->y(), 0.0f);
        ASSERT_FLOAT_EQ(se2_configuration->yaw(), 0.0f);
    }

    TEST(ConfigurationBase, LoadSe3State)
    {
        std::ifstream in("data/geometric_planning/ompl/se3_state.json");
        nlohmann::json j;
        in >> j;
        auto configuration = Se3StateOmplConfiguration::deserializeFromJson(j);
        ASSERT_EQ(configuration->configurationType(), ConfigurationType::e_ompl);
        ASSERT_EQ(configuration->goalType(), OmplGoalType::e_state);
        ASSERT_EQ(configuration->stateSpaceType(), OmplStateSpaceType::e_se3);
        ASSERT_FLOAT_EQ(configuration->x(), 1.0f);
        ASSERT_FLOAT_EQ(configuration->y(), 0.0f);
        ASSERT_FLOAT_EQ(configuration->z(), 0.0f);
        ASSERT_FLOAT_EQ(configuration->qx(), 0.0f);
        ASSERT_FLOAT_EQ(configuration->qy(), 0.0f);
        ASSERT_FLOAT_EQ(configuration->qz(), 0.0f);
        ASSERT_FLOAT_EQ(configuration->qw(), 1.0f);
    }

    TEST(ConfigurationBase, LoadSe3OmplConfiguration_Se3State)
    {
        std::ifstream in("data/geometric_planning/ompl/se3_state.json");
        nlohmann::json j;
        in >> j;
        auto configuration     = Se3OmplConfiguration::deserializeFromJson(j);
        auto se3_configuration = std::dynamic_pointer_cast<const Se3StateOmplConfiguration>(configuration);
        ASSERT_EQ(se3_configuration->configurationType(), ConfigurationType::e_ompl);
        ASSERT_EQ(se3_configuration->goalType(), OmplGoalType::e_state);
        ASSERT_EQ(se3_configuration->stateSpaceType(), OmplStateSpaceType::e_se3);
        ASSERT_FLOAT_EQ(se3_configuration->x(), 1.0f);
        ASSERT_FLOAT_EQ(se3_configuration->y(), 0.0f);
        ASSERT_FLOAT_EQ(se3_configuration->z(), 0.0f);
        ASSERT_FLOAT_EQ(se3_configuration->qx(), 0.0f);
        ASSERT_FLOAT_EQ(se3_configuration->qy(), 0.0f);
        ASSERT_FLOAT_EQ(se3_configuration->qz(), 0.0f);
        ASSERT_FLOAT_EQ(se3_configuration->qw(), 1.0f);
    }

    TEST(ConfigurationBase, LoadOmplConfiguration_Se3State)
    {
        std::ifstream in("data/geometric_planning/ompl/se3_state.json");
        nlohmann::json j;
        in >> j;
        auto configuration     = OmplConfiguration::deserializeFromJson(j);
        auto se3_configuration = std::dynamic_pointer_cast<const Se3StateOmplConfiguration>(configuration);
        ASSERT_EQ(se3_configuration->configurationType(), ConfigurationType::e_ompl);
        ASSERT_EQ(se3_configuration->goalType(), OmplGoalType::e_state);
        ASSERT_EQ(se3_configuration->stateSpaceType(), OmplStateSpaceType::e_se3);
        ASSERT_FLOAT_EQ(se3_configuration->x(), 1.0f);
        ASSERT_FLOAT_EQ(se3_configuration->y(), 0.0f);
        ASSERT_FLOAT_EQ(se3_configuration->z(), 0.0f);
        ASSERT_FLOAT_EQ(se3_configuration->qx(), 0.0f);
        ASSERT_FLOAT_EQ(se3_configuration->qy(), 0.0f);
        ASSERT_FLOAT_EQ(se3_configuration->qz(), 0.0f);
        ASSERT_FLOAT_EQ(se3_configuration->qw(), 1.0f);
    }

    TEST(ConfigurationBase, LoadConfigurationbase_Se3State)
    {
        std::ifstream in("data/geometric_planning/ompl/se3_state.json");
        nlohmann::json j;
        in >> j;
        auto configuration     = ConfigurationBase::deserializeFromJson(j);
        auto se3_configuration = std::dynamic_pointer_cast<const Se3StateOmplConfiguration>(configuration);
        ASSERT_EQ(se3_configuration->configurationType(), ConfigurationType::e_ompl);
        ASSERT_EQ(se3_configuration->goalType(), OmplGoalType::e_state);
        ASSERT_EQ(se3_configuration->stateSpaceType(), OmplStateSpaceType::e_se3);
        ASSERT_FLOAT_EQ(se3_configuration->x(), 1.0f);
        ASSERT_FLOAT_EQ(se3_configuration->y(), 0.0f);
        ASSERT_FLOAT_EQ(se3_configuration->z(), 0.0f);
        ASSERT_FLOAT_EQ(se3_configuration->qx(), 0.0f);
        ASSERT_FLOAT_EQ(se3_configuration->qy(), 0.0f);
        ASSERT_FLOAT_EQ(se3_configuration->qz(), 0.0f);
        ASSERT_FLOAT_EQ(se3_configuration->qw(), 1.0f);
    }

    // todo(Andrew): LoadGridCell
    // todo(Andrew): LoadGraphNode
}  // namespace grstapse::unittests