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
#include <functional>
#include <tuple>
#include <utility>

// External
//// Boost
#include <boost/functional/hash.hpp>
//// External
#include <Eigen/Core>

namespace std
{
    template <typename First, typename Second>
    struct hash<std::pair<First, Second>>
    {
        size_t operator()(const std::pair<First, Second>& p) const
        {
            size_t seed = 0;
            boost::hash_combine(seed, p.first);
            boost::hash_combine(seed, p.second);
            return seed;
        }
    };

    template <typename Scalar, int Rows, int Cols>
    struct hash<Eigen::Matrix<Scalar, Rows, Cols>>
    {
        size_t operator()(const Eigen::Matrix<Scalar, Rows, Cols>& matrix) const
        {
            const unsigned int num_rows = matrix.rows();
            const unsigned int num_cols = matrix.cols();
            size_t seed                 = 0;
            boost::hash_combine(seed, matrix.rows());
            boost::hash_combine(seed, matrix.cols());
            for(unsigned int row_nr = 0; row_nr < num_rows; ++row_nr)
            {
                for(unsigned int col_nr = 0; col_nr < num_cols; ++col_nr)
                {
                    boost::hash_combine(seed, matrix(row_nr, col_nr));
                }
            }
            return seed;
        }
    };

    template <typename Scalar, int Rows, int Cols>
    struct hash<Eigen::Block<Scalar, Rows, Cols>>
    {
        size_t operator()(const Eigen::Block<Scalar, Rows, Cols>& block) const
        {
            const unsigned int num_rows = block.rows();
            const unsigned int num_cols = block.cols();
            size_t seed                 = 0;
            boost::hash_combine(seed, block.rows());
            boost::hash_combine(seed, block.cols());
            for(unsigned int row_nr = 0; row_nr < num_rows; ++row_nr)
            {
                for(unsigned int col_nr = 0; col_nr < num_cols; ++col_nr)
                {
                    boost::hash_combine(seed, block(row_nr, col_nr));
                }
            }
            return seed;
        }
    };

}  // namespace std