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
#include <memory>
// External
#include <Eigen/Core>
#include <gtest/gtest.h>
// Project
#include <grstapse/core.hpp>

namespace grstapse::unittests
{
    /**!
     * Simple Matrix Multiply
     */
    TEST(RobotTraitsMatrixReduction, SimpleMatrixMultiply)
    {
        RobotTraitsMatrixReduction reduction;
        Eigen::MatrixXf allocation(2, 2);
        allocation << 1.0f, 0.0f, 1.0f, 1.0f;  // [ [1 0] [1 1] ]
        Eigen::MatrixXf robot_traits_matrix(2, 2);
        robot_traits_matrix << 1.0f, 2.0f, 3.0f, 4.0f;  // [ [1 2] [3 4] ]
        Eigen::MatrixXf result = reduction.reduce(allocation, robot_traits_matrix);

        Eigen::MatrixXf correct_result(2, 2);
        correct_result << 1.0f, 2.0f, 4.0f, 6.0f;  // [ [1 2] [4 6] ]
        ASSERT_EQ(result, correct_result);
    }

    /**!
     * Only summation (effectively matrix multiply)
     */
    TEST(RobotTraitsMatrixReduction, OnlySummation)
    {
        std::vector<std::vector<TraitsMatrixReductionTypes>> reduction_types = {
            {TraitsMatrixReductionTypes::e_summation, TraitsMatrixReductionTypes::e_summation},
            {TraitsMatrixReductionTypes::e_summation, TraitsMatrixReductionTypes::e_summation}};

        RobotTraitsMatrixReduction reduction(reduction_types);
        Eigen::MatrixXf allocation(2, 2);
        allocation << 1.0f, 0.0f, 1.0f, 1.0f;  // [ [1 0] [1 1] ]
        Eigen::MatrixXf robot_traits_matrix(2, 2);
        robot_traits_matrix << 1.0f, 2.0f, 3.0f, 4.0f;  // [ [1 2] [3 4] ]
        Eigen::MatrixXf result = reduction.reduce(allocation, robot_traits_matrix);

        Eigen::MatrixXf correct_result(2, 2);
        correct_result << 1.0f, 2.0f, 4.0f, 6.0f;  // [ [1 2] [4 6] ]
        ASSERT_EQ(result, correct_result);
    }

    /**!
     * Only summation on Custom Function Constructor
     */
    TEST(RobotTraitsMatrixReduction, OnlySummationCustomFunctionConstructor)
    {
        std::vector<std::vector<TraitsMatrixReductionTypes>> reduction_types = {
            {TraitsMatrixReductionTypes::e_summation, TraitsMatrixReductionTypes::e_summation},
            {TraitsMatrixReductionTypes::e_summation, TraitsMatrixReductionTypes::e_summation}};

        RobotTraitsMatrixReduction::CustomFunctionMap m;

        RobotTraitsMatrixReduction reduction(reduction_types, m);
        Eigen::MatrixXf allocation(2, 2);
        allocation << 1.0f, 0.0f, 1.0f, 1.0f;  // [ [1 0] [1 1] ]
        Eigen::MatrixXf robot_traits_matrix(2, 2);
        robot_traits_matrix << 1.0f, 2.0f, 3.0f, 4.0f;  // [ [1 2] [3 4] ]
        Eigen::MatrixXf result = reduction.reduce(allocation, robot_traits_matrix);

        Eigen::MatrixXf correct_result(2, 2);
        correct_result << 1.0f, 2.0f, 4.0f, 6.0f;  // [ [1 2] [4 6] ]
        ASSERT_EQ(result, correct_result);
    }

    /**!
     * Only summation on Custom Function Constructor with Custom Function
     *
     * \note Should throw an error
     */
    TEST(RobotTraitsMatrixReduction, OnlySummationCustomFunctionConstructorWithCustomFunction)
    {
        std::vector<std::vector<TraitsMatrixReductionTypes>> reduction_types = {
            {TraitsMatrixReductionTypes::e_summation, TraitsMatrixReductionTypes::e_summation},
            {TraitsMatrixReductionTypes::e_summation, TraitsMatrixReductionTypes::e_summation}};

        RobotTraitsMatrixReduction::CustomFunctionMap m = {
            {{0, 0}, std::make_shared<ThresholdCumulativeFunction>(0.5)}};

        ASSERT_ANY_THROW(RobotTraitsMatrixReduction(reduction_types, m));
    }

    /**!
     * Only product
     */
    TEST(RobotTraitsMatrixReduction, OnlyProduct)
    {
        std::vector<std::vector<TraitsMatrixReductionTypes>> reduction_types = {
            {TraitsMatrixReductionTypes::e_product, TraitsMatrixReductionTypes::e_product},
            {TraitsMatrixReductionTypes::e_product, TraitsMatrixReductionTypes::e_product}};

        RobotTraitsMatrixReduction reduction(reduction_types);
        Eigen::MatrixXf allocation(2, 2);
        allocation << 1.0f, 0.0f, 1.0f, 1.0f;  // [ [1 0] [1 1] ]
        Eigen::MatrixXf robot_traits_matrix(2, 2);
        robot_traits_matrix << 1.0f, 2.0f, 3.0f, 4.0f;  // [ [1 2] [3 4] ]
        Eigen::MatrixXf result = reduction.reduce(allocation, robot_traits_matrix);

        Eigen::MatrixXf correct_result(2, 2);
        correct_result << 1.0f, 2.0f, 3.0f, 8.0f;  // [ [1 2] [3 8] ]
        ASSERT_EQ(result, correct_result);
    }

    /**!
     * Only product on Custom Function Constructor
     */
    TEST(RobotTraitsMatrixReduction, OnlyProductCustomFunctionConstructor)
    {
        std::vector<std::vector<TraitsMatrixReductionTypes>> reduction_types = {
            {TraitsMatrixReductionTypes::e_product, TraitsMatrixReductionTypes::e_product},
            {TraitsMatrixReductionTypes::e_product, TraitsMatrixReductionTypes::e_product}};
        RobotTraitsMatrixReduction::CustomFunctionMap m;

        RobotTraitsMatrixReduction reduction(reduction_types, m);
        Eigen::MatrixXf allocation(2, 2);
        allocation << 1.0f, 0.0f, 1.0f, 1.0f;  // [ [1 0] [1 1] ]
        Eigen::MatrixXf robot_traits_matrix(2, 2);
        robot_traits_matrix << 1.0f, 2.0f, 3.0f, 4.0f;  // [ [1 2] [3 4] ]
        Eigen::MatrixXf result = reduction.reduce(allocation, robot_traits_matrix);

        Eigen::MatrixXf correct_result(2, 2);
        correct_result << 1.0f, 2.0f, 3.0f, 8.0f;  // [ [1 2] [3 8] ]
        ASSERT_EQ(result, correct_result);
    }

    /**!
     * Only product on Custom Function Constructor with Custom Function
     *
     * \note Should throw an error
     */
    TEST(RobotTraitsMatrixReduction, OnlyProductCustomFunctionConstructorWithCustomFunction)
    {
        std::vector<std::vector<TraitsMatrixReductionTypes>> reduction_types = {
            {TraitsMatrixReductionTypes::e_product, TraitsMatrixReductionTypes::e_product},
            {TraitsMatrixReductionTypes::e_product, TraitsMatrixReductionTypes::e_product}};
        RobotTraitsMatrixReduction::CustomFunctionMap m = {
            {{0, 0}, std::make_shared<ThresholdCumulativeFunction>(0.5)}};

        ASSERT_ANY_THROW(RobotTraitsMatrixReduction(reduction_types, m));
    }

    /**!
     * Only minimum
     */
    TEST(RobotTraitsMatrixReduction, OnlyMinimum)
    {
        std::vector<std::vector<TraitsMatrixReductionTypes>> reduction_types = {
            {TraitsMatrixReductionTypes::e_minimum, TraitsMatrixReductionTypes::e_minimum},
            {TraitsMatrixReductionTypes::e_minimum, TraitsMatrixReductionTypes::e_minimum}};

        RobotTraitsMatrixReduction reduction(reduction_types);
        Eigen::MatrixXf allocation(2, 2);
        allocation << 1.0f, 0.0f, 1.0f, 1.0f;  // [ [1 0] [1 1] ]
        Eigen::MatrixXf robot_traits_matrix(2, 2);
        robot_traits_matrix << 1.0f, 2.0f, 3.0f, 4.0f;  // [ [1 2] [3 4] ]
        Eigen::MatrixXf result = reduction.reduce(allocation, robot_traits_matrix);

        Eigen::MatrixXf correct_result(2, 2);
        correct_result << 1.0f, 2.0f, 1.0f, 2.0f;  // [ [1 2] [1 2] ]
        ASSERT_EQ(result, correct_result);
    }

    /**!
     * Only minimum on Custom Function Constructor
     */
    TEST(RobotTraitsMatrixReduction, OnlyMinimumCustomFunctionConstructor)
    {
        std::vector<std::vector<TraitsMatrixReductionTypes>> reduction_types = {
            {TraitsMatrixReductionTypes::e_minimum, TraitsMatrixReductionTypes::e_minimum},
            {TraitsMatrixReductionTypes::e_minimum, TraitsMatrixReductionTypes::e_minimum}};
        RobotTraitsMatrixReduction::CustomFunctionMap m;

        RobotTraitsMatrixReduction reduction(reduction_types, m);
        Eigen::MatrixXf allocation(2, 2);
        allocation << 1.0f, 0.0f, 1.0f, 1.0f;  // [ [1 0] [1 1] ]
        Eigen::MatrixXf robot_traits_matrix(2, 2);
        robot_traits_matrix << 1.0f, 2.0f, 3.0f, 4.0f;  // [ [1 2] [3 4] ]
        Eigen::MatrixXf result = reduction.reduce(allocation, robot_traits_matrix);

        Eigen::MatrixXf correct_result(2, 2);
        correct_result << 1.0f, 2.0f, 1.0f, 2.0f;  // [ [1 2] [1 2] ]
        ASSERT_EQ(result, correct_result);
    }

    /**!
     * Only minimum on Custom Function Constructor with Custom Function
     *
     * \note Should throw an error
     */
    TEST(RobotTraitsMatrixReduction, OnlyMinimumCustomFunctionConstructorWithCustomFunction)
    {
        std::vector<std::vector<TraitsMatrixReductionTypes>> reduction_types = {
            {TraitsMatrixReductionTypes::e_minimum, TraitsMatrixReductionTypes::e_minimum},
            {TraitsMatrixReductionTypes::e_minimum, TraitsMatrixReductionTypes::e_minimum}};
        RobotTraitsMatrixReduction::CustomFunctionMap m = {
            {{0, 0}, std::make_shared<ThresholdCumulativeFunction>(0.5)}};

        ASSERT_ANY_THROW(RobotTraitsMatrixReduction(reduction_types, m));
    }

    /**!
     * Only maximum
     */
    TEST(RobotTraitsMatrixReduction, OnlyMaximum)
    {
        std::vector<std::vector<TraitsMatrixReductionTypes>> reduction_types = {
            {TraitsMatrixReductionTypes::e_maximum, TraitsMatrixReductionTypes::e_maximum},
            {TraitsMatrixReductionTypes::e_maximum, TraitsMatrixReductionTypes::e_maximum}};

        RobotTraitsMatrixReduction reduction(reduction_types);
        Eigen::MatrixXf allocation(2, 2);
        allocation << 1.0f, 0.0f, 1.0f, 1.0f;  // [ [1 0] [1 1] ]
        Eigen::MatrixXf robot_traits_matrix(2, 2);
        robot_traits_matrix << 1.0f, 2.0f, 3.0f, 4.0f;  // [ [1 2] [3 4] ]
        Eigen::MatrixXf result = reduction.reduce(allocation, robot_traits_matrix);

        Eigen::MatrixXf correct_result(2, 2);
        correct_result << 1.0f, 2.0f, 3.0f, 4.0f;  // [ [1 2] [3 4] ]
        ASSERT_EQ(result, correct_result);
    }

    /**!
     * Only maximum on Custom Function Constructor
     */
    TEST(RobotTraitsMatrixReduction, OnlyMaximumCustomFunctionConstructor)
    {
        std::vector<std::vector<TraitsMatrixReductionTypes>> reduction_types = {
            {TraitsMatrixReductionTypes::e_maximum, TraitsMatrixReductionTypes::e_maximum},
            {TraitsMatrixReductionTypes::e_maximum, TraitsMatrixReductionTypes::e_maximum}};
        RobotTraitsMatrixReduction::CustomFunctionMap m;

        RobotTraitsMatrixReduction reduction(reduction_types, m);
        Eigen::MatrixXf allocation(2, 2);
        allocation << 1.0f, 0.0f, 1.0f, 1.0f;  // [ [1 0] [1 1] ]
        Eigen::MatrixXf robot_traits_matrix(2, 2);
        robot_traits_matrix << 1.0f, 2.0f, 3.0f, 4.0f;  // [ [1 2] [3 4] ]
        Eigen::MatrixXf result = reduction.reduce(allocation, robot_traits_matrix);

        Eigen::MatrixXf correct_result(2, 2);
        correct_result << 1.0f, 2.0f, 3.0f, 4.0f;  // [ [1 2] [3 4] ]
        ASSERT_EQ(result, correct_result);
    }

    /**!
     * Only maximum on Custom Function Constructor with Custom Function
     *
     * \note Should throw an error
     */
    TEST(RobotTraitsMatrixReduction, OnlyMaximumCustomFunctionConstructorWithCustomFunction)
    {
        std::vector<std::vector<TraitsMatrixReductionTypes>> reduction_types = {
            {TraitsMatrixReductionTypes::e_maximum, TraitsMatrixReductionTypes::e_maximum},
            {TraitsMatrixReductionTypes::e_maximum, TraitsMatrixReductionTypes::e_maximum}};
        RobotTraitsMatrixReduction::CustomFunctionMap m = {
            {{0, 0}, std::make_shared<ThresholdCumulativeFunction>(0.5)}};

        ASSERT_ANY_THROW(RobotTraitsMatrixReduction(reduction_types, m));
    }

    /**!
     * Threshold Cumulative Function
     */
    TEST(RobotTraitsMatrixReduction, ThresholdCumulativeFunction)
    {
        std::vector<std::vector<TraitsMatrixReductionTypes>> reduction_types = {
            {TraitsMatrixReductionTypes::e_custom, TraitsMatrixReductionTypes::e_custom},
            {TraitsMatrixReductionTypes::e_custom, TraitsMatrixReductionTypes::e_custom}};
        RobotTraitsMatrixReduction::CustomFunctionMap m = {
            {{0, 0}, std::make_shared<ThresholdCumulativeFunction>(0.5)},
            {{0, 1}, std::make_shared<ThresholdCumulativeFunction>(0.5)},
            {{1, 0}, std::make_shared<ThresholdCumulativeFunction>(0.5)},
            {{1, 1}, std::make_shared<ThresholdCumulativeFunction>(0.5)},
        };

        RobotTraitsMatrixReduction reduction(reduction_types, m);
        Eigen::MatrixXf allocation(2, 2);
        allocation << 1.0f, 0.0f, 1.0f, 1.0f;  // [ [1 0] [1 1] ]
        Eigen::MatrixXf robot_traits_matrix(2, 2);
        robot_traits_matrix << 0.0f, 1.0f, 1.0f, 1.0f;  // [ [0 1] [1 1] ]
        Eigen::MatrixXf result = reduction.reduce(allocation, robot_traits_matrix);

        Eigen::MatrixXf correct_result(2, 2);
        correct_result << 0.0f, 1.0f, 1.0f, 2.0f;  // [ [0 1] [1 2] ]
        ASSERT_EQ(result, correct_result);
    }

    /**!
     * One of each
     */
    TEST(RobotTraitsMatrixReduction, OneOfEach)
    {
        std::vector<std::vector<TraitsMatrixReductionTypes>> reduction_types = {
            {TraitsMatrixReductionTypes::e_summation,
             TraitsMatrixReductionTypes::e_product,
             TraitsMatrixReductionTypes::e_minimum,
             TraitsMatrixReductionTypes::e_maximum,
             TraitsMatrixReductionTypes::e_custom}};
        RobotTraitsMatrixReduction::CustomFunctionMap m = {{{0, 4}, std::make_shared<ThresholdCumulativeFunction>(7)}};

        RobotTraitsMatrixReduction reduction(reduction_types, m);
        Eigen::MatrixXf allocation(1, 2);
        allocation << 1.0f, 1.0f;  // [ [1 1] ]
        Eigen::MatrixXf robot_traits_matrix(2, 5);
        robot_traits_matrix << 1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f, 7.0f, 8.0f, 9.0f,
            10.0f;  // [ [1 2 3 4 5] [6 7 8 9 10] ]
        Eigen::MatrixXf result = reduction.reduce(allocation, robot_traits_matrix);

        Eigen::MatrixXf correct_result(1, 5);
        correct_result << 7.0f, 14.0f, 3.0f, 9.0f, 1.0f;  // [ [7 14 3 9 1]  ]
        ASSERT_EQ(result, correct_result);
    }

    /**!
     * Load a matrix multiple RobotTraitsMatrixReduction from file
     */
    TEST(RobotTraitsMatrixReduction, LoadMatrixMultiply)
    {
        std::ifstream fin("data/task_allocation/robot_traits_matrix_reduction/matrix_multiply.json");
        nlohmann::json j;
        fin >> j;
        auto reduction = j.get<std::shared_ptr<RobotTraitsMatrixReduction>>();
        Eigen::MatrixXf allocation(2, 2);
        allocation << 1.0f, 0.0f, 1.0f, 1.0f;  // [ [1 0] [1 1] ]
        Eigen::MatrixXf robot_traits_matrix(2, 2);
        robot_traits_matrix << 1.0f, 2.0f, 3.0f, 4.0f;  // [ [1 2] [3 4] ]
        Eigen::MatrixXf result = reduction->reduce(allocation, robot_traits_matrix);

        Eigen::MatrixXf correct_result(2, 2);
        correct_result << 1.0f, 2.0f, 4.0f, 6.0f;  // [ [1 2] [4 6] ]
        ASSERT_EQ(result, correct_result);
    }

    /**!
     * Load a summation only RobotTraitsMatrixReduction from file
     */
    TEST(RobotTraitsMatrixReduction, LoadSummation)
    {
        std::ifstream fin("data/task_allocation/robot_traits_matrix_reduction/summation.json");
        nlohmann::json j;
        fin >> j;
        auto reduction = j.get<std::shared_ptr<RobotTraitsMatrixReduction>>();
        Eigen::MatrixXf allocation(2, 2);
        allocation << 1.0f, 0.0f, 1.0f, 1.0f;  // [ [1 0] [1 1] ]
        Eigen::MatrixXf robot_traits_matrix(2, 2);
        robot_traits_matrix << 1.0f, 2.0f, 3.0f, 4.0f;  // [ [1 2] [3 4] ]
        Eigen::MatrixXf result = reduction->reduce(allocation, robot_traits_matrix);

        Eigen::MatrixXf correct_result(2, 2);
        correct_result << 1.0f, 2.0f, 4.0f, 6.0f;  // [ [1 2] [4 6] ]
        ASSERT_EQ(result, correct_result);
    }

    /**!
     * Load a product only RobotTraitsMatrixReduction from file
     */
    TEST(RobotTraitsMatrixReduction, LoadProduct)
    {
        std::ifstream fin("data/task_allocation/robot_traits_matrix_reduction/product.json");
        nlohmann::json j;
        fin >> j;
        auto reduction = j.get<std::shared_ptr<RobotTraitsMatrixReduction>>();
        Eigen::MatrixXf allocation(2, 2);
        allocation << 1.0f, 0.0f, 1.0f, 1.0f;  // [ [1 0] [1 1] ]
        Eigen::MatrixXf robot_traits_matrix(2, 2);
        robot_traits_matrix << 1.0f, 2.0f, 3.0f, 4.0f;  // [ [1 2] [3 4] ]
        Eigen::MatrixXf result = reduction->reduce(allocation, robot_traits_matrix);

        Eigen::MatrixXf correct_result(2, 2);
        correct_result << 1.0f, 2.0f, 3.0f, 8.0f;  // [ [1 2] [3 8] ]
        ASSERT_EQ(result, correct_result);
    }

    /**!
     * Load a minimum only RobotTraitsMatrixReduction from file
     */
    TEST(RobotTraitsMatrixReduction, LoadMinimum)
    {
        std::ifstream fin("data/task_allocation/robot_traits_matrix_reduction/minimum.json");
        nlohmann::json j;
        fin >> j;
        auto reduction = j.get<std::shared_ptr<RobotTraitsMatrixReduction>>();
        Eigen::MatrixXf allocation(2, 2);
        allocation << 1.0f, 0.0f, 1.0f, 1.0f;  // [ [1 0] [1 1] ]
        Eigen::MatrixXf robot_traits_matrix(2, 2);
        robot_traits_matrix << 1.0f, 2.0f, 3.0f, 4.0f;  // [ [1 2] [3 4] ]
        Eigen::MatrixXf result = reduction->reduce(allocation, robot_traits_matrix);

        Eigen::MatrixXf correct_result(2, 2);
        correct_result << 1.0f, 2.0f, 1.0f, 2.0f;  // [ [1 2] [1 2] ]
        ASSERT_EQ(result, correct_result);
    }

    /**!
     * Load a maximum only RobotTraitsMatrixReduction from file
     */
    TEST(RobotTraitsMatrixReduction, LoadMaximum)
    {
        std::ifstream fin("data/task_allocation/robot_traits_matrix_reduction/maximum.json");
        nlohmann::json j;
        fin >> j;
        auto reduction = j.get<std::shared_ptr<RobotTraitsMatrixReduction>>();
        Eigen::MatrixXf allocation(2, 2);
        allocation << 1.0f, 0.0f, 1.0f, 1.0f;  // [ [1 0] [1 1] ]
        Eigen::MatrixXf robot_traits_matrix(2, 2);
        robot_traits_matrix << 1.0f, 2.0f, 3.0f, 4.0f;  // [ [1 2] [3 4] ]
        Eigen::MatrixXf result = reduction->reduce(allocation, robot_traits_matrix);

        Eigen::MatrixXf correct_result(2, 2);
        correct_result << 1.0f, 2.0f, 3.0f, 4.0f;  // [ [1 2] [3 4] ]
        ASSERT_EQ(result, correct_result);
    }

    /**!
     * Load a custom only RobotTraitsMatrixReduction from file
     */
    TEST(RobotTraitsMatrixReduction, LoadCustom)
    {
        std::ifstream fin("data/task_allocation/robot_traits_matrix_reduction/custom.json");
        nlohmann::json j;
        fin >> j;
        auto reduction = j.get<std::shared_ptr<RobotTraitsMatrixReduction>>();
        Eigen::MatrixXf allocation(2, 2);
        allocation << 1.0f, 0.0f, 1.0f, 1.0f;  // [ [1 0] [1 1] ]
        Eigen::MatrixXf robot_traits_matrix(2, 2);
        robot_traits_matrix << 0.0f, 1.0f, 1.0f, 1.0f;  // [ [0 1] [1 1] ]
        Eigen::MatrixXf result = reduction->reduce(allocation, robot_traits_matrix);

        Eigen::MatrixXf correct_result(2, 2);
        correct_result << 0.0f, 1.0f, 1.0f, 2.0f;  // [ [0 1] [1 2] ]
        ASSERT_EQ(result, correct_result);
    }

    /**!
     * Load a one of each RobotTraitsMatrixReduction from file
     */
    TEST(RobotTraitsMatrixReduction, LoadOneOfEach)
    {
        std::ifstream fin("data/task_allocation/robot_traits_matrix_reduction/one_of_each.json");
        nlohmann::json j;
        fin >> j;
        auto reduction = j.get<std::shared_ptr<RobotTraitsMatrixReduction>>();
        Eigen::MatrixXf allocation(1, 2);
        allocation << 1.0f, 1.0f;  // [ [1 1] ]
        Eigen::MatrixXf robot_traits_matrix(2, 5);
        robot_traits_matrix << 1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f, 7.0f, 8.0f, 9.0f,
            10.0f;  // [ [1 2 3 4 5] [6 7 8 9 10] ]
        Eigen::MatrixXf result = reduction->reduce(allocation, robot_traits_matrix);

        Eigen::MatrixXf correct_result(1, 5);
        correct_result << 7.0f, 14.0f, 3.0f, 9.0f, 1.0f;  // [ [7 14 3 9 1]  ]
        ASSERT_EQ(result, correct_result);
    }
}  // namespace grstapse::unittests