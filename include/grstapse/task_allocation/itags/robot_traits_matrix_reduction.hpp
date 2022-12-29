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
#pragma once

// Global
#include <tuple>
#include <vector>
// External
#include <Eigen/Core>
#include <nlohmann/json.hpp>
#include <robin_hood/robin_hood.hpp>
// Local
#include "grstapse/common/utilities/custom_hashings.hpp"
#include "grstapse/common/utilities/noncopyable.hpp"
#include "grstapse/task_allocation/itags/vector_reduction_function.hpp"

namespace grstapse
{
    enum class TraitsMatrixReductionTypes : uint8_t
    {
        e_summation,
        e_product,
        e_minimum,
        e_maximum,
        e_custom
    };
    NLOHMANN_JSON_SERIALIZE_ENUM(TraitsMatrixReductionTypes,
                                 {{TraitsMatrixReductionTypes::e_summation, "summation"},
                                  {TraitsMatrixReductionTypes::e_product, "product"},
                                  {TraitsMatrixReductionTypes::e_minimum, "minimum"},
                                  {TraitsMatrixReductionTypes::e_maximum, "maximum"},
                                  {TraitsMatrixReductionTypes::e_custom, "custom"}});

    /**!
     * Combines an allocation with the matrix of traits for an entire team of robot to create a matrix representing the
     * traits allocated to each task in a task network
     *
     * \note Supports more than just summation
     */
    class RobotTraitsMatrixReduction : public Noncopyable
    {
       public:
        using CustomFunctionMap =
            robin_hood::unordered_map<std::pair<unsigned int, unsigned int>, std::shared_ptr<VectorReductionFunction>>;

        //! \brief Constructor for matrix multiplication
        RobotTraitsMatrixReduction();

        //! \brief Constructor for various reduction types (except custom)
        RobotTraitsMatrixReduction(const std::vector<std::vector<TraitsMatrixReductionTypes>>& reduction_types);

        //! \brief Constructor for various reduction types including custom
        RobotTraitsMatrixReduction(const std::vector<std::vector<TraitsMatrixReductionTypes>>& reduction_types,
                                   const CustomFunctionMap& custom_functions);

        /**!
         * Combines an allocation with the matrix of traits for an entire team of robot to create a matrix representing
         * the traits allocated to each task in a task network
         *
         * \param The allocation for the coalition
         * \param robot_traits_matrix A matrix representing the traits of the entire team
         *
         * \returns A matrix representing the traits allocated to each task in a task network
         */
        [[nodiscard]] Eigen::MatrixXf reduce(const Eigen::MatrixXf& allocation,
                                             const Eigen::MatrixXf& robot_traits_matrix) const;

       protected:
        /**!
         * \brief allocation * robot_traits_matrix
         *
         * \note Only used when all elements of m_reduction_types are e_summation
         */
        [[nodiscard]] Eigen::MatrixXf reduce_MatrixMultiply(const Eigen::MatrixXf& allocation,
                                                            const Eigen::MatrixXf& robot_traits_matrix) const;

        //! \note Only used for benchmarking
        [[nodiscard]] Eigen::MatrixXf reduce_Loop(const Eigen::MatrixXf& allocation,
                                                  const Eigen::MatrixXf& robot_traits_matrix) const;

        //! \brief Fast reductions
        [[nodiscard]] Eigen::MatrixXf reduce_EigenReduction(const Eigen::MatrixXf& allocation,
                                                            const Eigen::MatrixXf& robot_traits_matrix) const;

       private:
        bool m_matrix_multiply;  // true only if all elements of m_reduction_types are e_summation
        std::vector<std::vector<TraitsMatrixReductionTypes>> m_reduction_types;
        CustomFunctionMap m_custom;

        friend void from_json(const nlohmann::json& j, RobotTraitsMatrixReduction& r);
    };

    void from_json(const nlohmann::json& j, RobotTraitsMatrixReduction& r);
}  // namespace grstapse