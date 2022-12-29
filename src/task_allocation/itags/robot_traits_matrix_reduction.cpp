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
#include "grstapse/task_allocation/itags/robot_traits_matrix_reduction.hpp"

// External
#include <fmt/format.h>
#include <magic_enum/magic_enum.hpp>
// Local
#include "grstapse/common/utilities/error.hpp"
#include "grstapse/common/utilities/logger.hpp"

namespace grstapse
{
    RobotTraitsMatrixReduction::RobotTraitsMatrixReduction()
        : m_matrix_multiply(true)
        , m_reduction_types()
        , m_custom()
    {}

    RobotTraitsMatrixReduction::RobotTraitsMatrixReduction(
        const std::vector<std::vector<TraitsMatrixReductionTypes>>& reduction_types)
        : m_reduction_types(reduction_types)
        , m_custom()
    {
        // If all reduction types are summation then matrix multiplication is the fastest method to reduce the
        // robot trait matrix to those allocated
        m_matrix_multiply = true;
        for(const std::vector<TraitsMatrixReductionTypes>& inner: reduction_types)
        {
            for(TraitsMatrixReductionTypes type: inner)
            {
                if(type != TraitsMatrixReductionTypes::e_summation)
                {
                    m_matrix_multiply = false;
                }
                if(type == TraitsMatrixReductionTypes::e_custom)
                {
                    throw createLogicError("Custom reduction must provide a function");
                }
            }
        }
    }

    RobotTraitsMatrixReduction::RobotTraitsMatrixReduction(
        const std::vector<std::vector<TraitsMatrixReductionTypes>>& reduction_types,
        const CustomFunctionMap& custom_functions)
        : m_reduction_types(reduction_types)
        , m_custom(custom_functions)
    {
        // If all reduction types are summation then matrix multiplication is the fastest method to reduce the
        // robot trait matrix to those allocated
        m_matrix_multiply = true;
        for(const std::vector<TraitsMatrixReductionTypes>& inner: reduction_types)
        {
            for(TraitsMatrixReductionTypes type: inner)
            {
                if(type != TraitsMatrixReductionTypes::e_summation)
                {
                    m_matrix_multiply = false;
                    break;
                }
            }
            if(m_matrix_multiply == false)
            {
                break;
            }
        }
        const unsigned int num_tasks  = reduction_types.size();
        const unsigned int num_traits = reduction_types[0].size();
        for(auto& [key, value]: custom_functions)
        {
            if(key.first >= num_tasks || key.second >= num_traits)
            {
                const std::string error_message = fmt::format("({}, {}) is outside the bounds ({}, {})",
                                                              key.first,
                                                              key.second,
                                                              num_tasks,
                                                              num_traits);
                throw createLogicError(error_message);
            }
            if(m_reduction_types[key.first][key.second] != TraitsMatrixReductionTypes::e_custom || !value)
            {
                const std::string error_message =
                    fmt::format("({}, {}) is of type {}",
                                key.first,
                                key.second,
                                magic_enum::enum_name(m_reduction_types[key.first][key.second]));
                throw createLogicError(error_message);
            }
        }
        if(m_matrix_multiply)
        {
            Logger::warn("Using the custom reduction constructor for a matrix multiply reduction?");
        }
    }

    Eigen::MatrixXf RobotTraitsMatrixReduction::reduce(const Eigen::MatrixXf& allocation,
                                                       const Eigen::MatrixXf& robot_traits_matrix) const
    {
        if(m_matrix_multiply)
        {
            return reduce_MatrixMultiply(allocation, robot_traits_matrix);
        }
        else
        {
            return reduce_EigenReduction(allocation, robot_traits_matrix);
        }

        // reduce_Loop is never used because it is much slower
    }

    Eigen::MatrixXf RobotTraitsMatrixReduction::reduce_MatrixMultiply(const Eigen::MatrixXf& allocation,
                                                                      const Eigen::MatrixXf& robot_traits_matrix) const
    {
        return allocation * robot_traits_matrix;
    }

    Eigen::MatrixXf RobotTraitsMatrixReduction::reduce_Loop(const Eigen::MatrixXf& allocation,
                                                            const Eigen::MatrixXf& robot_traits_matrix) const
    {
        const unsigned int num_tasks = allocation.rows();
        if(num_tasks != m_reduction_types.size())
        {
            throw createLogicError("Number of tasks doesn't match with reduction types");
        }
        const unsigned int num_robots = allocation.cols();
        const unsigned int num_traits = robot_traits_matrix.cols();
        if(num_traits != m_reduction_types[0].size())
        {
            throw createLogicError("Number of traits doesn't match with reduction types");
        }

        Eigen::MatrixXf rv = Eigen::MatrixXf::Zero(num_tasks, num_traits);
        for(unsigned int task_nr = 0; task_nr < num_tasks; ++task_nr)
        {
            for(unsigned int trait_nr = 0; trait_nr < num_traits; ++trait_nr)
            {
                for(unsigned int robot_nr = 0; robot_nr < num_robots; ++robot_nr)
                {
                    if(allocation(task_nr, robot_nr))
                    {
                        switch(m_reduction_types[task_nr][trait_nr])
                        {
                            case TraitsMatrixReductionTypes::e_summation:
                                rv(task_nr, trait_nr) += robot_traits_matrix(robot_nr, trait_nr);
                                break;
                            case TraitsMatrixReductionTypes::e_product:
                                rv(task_nr, trait_nr) *= robot_traits_matrix(robot_nr, trait_nr);
                                break;
                            case TraitsMatrixReductionTypes::e_minimum:
                                rv(task_nr, trait_nr) =
                                    std::min(rv(task_nr, trait_nr), robot_traits_matrix(robot_nr, trait_nr));
                                break;
                            case TraitsMatrixReductionTypes::e_maximum:
                                rv(task_nr, trait_nr) =
                                    std::max(rv(task_nr, trait_nr), robot_traits_matrix(robot_nr, trait_nr));
                                break;
                        }
                    }
                }
            }
        }

        return rv;
    }

    Eigen::MatrixXf RobotTraitsMatrixReduction::reduce_EigenReduction(const Eigen::MatrixXf& allocation,
                                                                      const Eigen::MatrixXf& robot_traits_matrix) const
    {
        const unsigned int num_tasks = allocation.rows();
        if(num_tasks != m_reduction_types.size())
        {
            throw createLogicError("Number of tasks doesn't match with reduction types");
        }
        const unsigned int num_traits = robot_traits_matrix.cols();
        if(num_traits != m_reduction_types[0].size())
        {
            throw createLogicError("Number of traits doesn't match with reduction types");
        }

        Eigen::MatrixXf rv = Eigen::MatrixXf::Zero(num_tasks, num_traits);
        for(unsigned int task_nr = 0; task_nr < num_tasks; ++task_nr)
        {
            // Create the allocated traits matrix for the task
            const Eigen::VectorXf& task_allocation_vector = allocation.row(task_nr);
            Eigen::VectorXi is_selected                   = (task_allocation_vector.array() > 0.5).cast<int>();
            Eigen::MatrixXf allocated_traits_matrix(is_selected.sum(), robot_traits_matrix.cols());
            unsigned int row_nr = 0;
            for(unsigned int i = 0, i_end = robot_traits_matrix.rows(); i < i_end; ++i)
            {
                if(is_selected[i])
                {
                    allocated_traits_matrix.row(row_nr++) = robot_traits_matrix.row(i);
                }
            }

            for(unsigned int trait_nr = 0; trait_nr < num_traits; ++trait_nr)
            {
                switch(m_reduction_types[task_nr][trait_nr])
                {
                    case TraitsMatrixReductionTypes::e_summation:
                        rv(task_nr, trait_nr) = allocated_traits_matrix.col(trait_nr).sum();
                        break;
                    case TraitsMatrixReductionTypes::e_product:
                        rv(task_nr, trait_nr) = allocated_traits_matrix.col(trait_nr).prod();
                        break;
                    case TraitsMatrixReductionTypes::e_minimum:
                        rv(task_nr, trait_nr) = allocated_traits_matrix.col(trait_nr).minCoeff();
                        break;
                    case TraitsMatrixReductionTypes::e_maximum:
                        rv(task_nr, trait_nr) = allocated_traits_matrix.col(trait_nr).maxCoeff();
                        break;
                    case TraitsMatrixReductionTypes::e_custom:
                        rv(task_nr, trait_nr) =
                            m_custom.at(std::pair(task_nr, trait_nr))->reduce(allocated_traits_matrix.col(trait_nr));
                        break;
                }
            }
        }

        return rv;
    }

    void from_json(const nlohmann::json& j, RobotTraitsMatrixReduction& r)
    {
        r.m_matrix_multiply = true;
        if(j.find("reduction_types") != j.end())
        {
            j.at("reduction_types").get_to(r.m_reduction_types);
            for(const std::vector<TraitsMatrixReductionTypes>& inner: r.m_reduction_types)
            {
                for(TraitsMatrixReductionTypes type: inner)
                {
                    if(type != TraitsMatrixReductionTypes::e_summation)
                    {
                        r.m_matrix_multiply = false;
                        break;
                    }
                }
                if(r.m_matrix_multiply == false)
                {
                    break;
                }
            }

            if(j.find("custom_functions") != j.end())
            {
                const unsigned int num_task   = r.m_reduction_types.size();
                const unsigned int num_traits = r.m_reduction_types[0].size();
                for(const nlohmann::json& custom_function_j: j.at("custom_functions"))
                {
                    unsigned int row = custom_function_j.at("row");
                    unsigned int col = custom_function_j.at("col");
                    if(row >= num_task || col >= num_traits)
                    {
                        const std::string error_message =
                            fmt::format("({}, {}) is outside the bounds ({}, {})", row, col, num_task, num_traits);
                        throw createLogicError(error_message);
                    }
                    if(r.m_reduction_types[row][col] != TraitsMatrixReductionTypes::e_custom)
                    {
                        const std::string error_message =
                            fmt::format("({}, {}) is of type {}",
                                        row,
                                        col,
                                        magic_enum::enum_name(r.m_reduction_types[row][col]));
                        throw createLogicError(error_message);
                    }
                    std::shared_ptr<VectorReductionFunction> function =
                        VectorReductionFunction::load(custom_function_j.at("parameters"));
                    r.m_custom[std::pair(row, col)] = function;
                }
            }
        }
    }
}  // namespace grstapse