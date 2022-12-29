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
// External
#include <Eigen/Core>
#include <gtest/gtest.h>
// Local
#include <grstapse/common/utilities/custom_hashings.hpp>

namespace grstapse::unittests
{
    TEST(CustomHashing, TransposeMatrix)
    {
        Eigen::MatrixXf m1(3, 3);
        m1 << 1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f, 7.0f, 8.0f, 9.0f;
        unsigned int h1 = std::hash<Eigen::MatrixXf>()(m1);

        Eigen::MatrixXf m2(3, 3);
        m2 << 1.0f, 4.0f, 7.0f, 2.0f, 5.0f, 8.0f, 3.0f, 6.0f, 9.0f;
        unsigned int h2 = std::hash<Eigen::MatrixXf>()(m2);

        ASSERT_NE(h1, h2);
    }
}  // namespace grstapse::unittests