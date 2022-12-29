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
#include "grstapse/common/utilities/custom_json_conversions.hpp"
#include "grstapse/geometric_planning/ompl/ompl_enums.hpp"
#include "grstapse/geometric_planning/pgm_environment.hpp"

namespace grstapse::unittests
{
    TEST(PgmEnvironment, Load)
    {
        std::ifstream fin("data/geometric_planning/environments/pgm.json");
        nlohmann::json j;
        fin >> j;
        auto environment = j.get<std::shared_ptr<PgmEnvironment>>();

        ASSERT_FLOAT_EQ(environment->resolution(), 0.05);
        ASSERT_FLOAT_EQ(environment->minX(), -51.224998);
        // ASSERT_NEAR(environment->maxX(), 51.224998, 1e-3);
        ASSERT_FLOAT_EQ(environment->minY(), -51.224998);
        // ASSERT_NEAR(environment->maxY(), 51.224998, 1e-3);
        ASSERT_EQ(environment->stateSpaceType(), OmplStateSpaceType::e_se2);
    }

    TEST(PgmEnvironment, LoadDublin)
    {
        std::ifstream fin("data/geometric_planning/environments/pgm_dubins.json");
        nlohmann::json j;
        fin >> j;
        auto environment = j.get<std::shared_ptr<PgmEnvironment>>();

        ASSERT_FLOAT_EQ(environment->resolution(), 0.05);
        ASSERT_FLOAT_EQ(environment->minX(), -51.224998);
        // ASSERT_NEAR(environment->maxX(), 51.224998, 1e-3);
        ASSERT_FLOAT_EQ(environment->minY(), -51.224998);
        // ASSERT_NEAR(environment->maxY(), 51.224998, 1e-3);
        ASSERT_EQ(environment->stateSpaceType(), OmplStateSpaceType::e_se2);
    }
}  // namespace grstapse::unittests