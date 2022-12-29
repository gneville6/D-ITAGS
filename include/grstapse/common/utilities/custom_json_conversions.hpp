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
#include <memory>
// External
#include <Eigen/Core>
#include <nlohmann/json.hpp>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/spaces/SO3StateSpace.h>
#include <ompl/geometric/PathGeometric.h>
// Local
#include "grstapse/common/utilities/constants.hpp"
#include "grstapse/common/utilities/error.hpp"

//! \file Several functions for converting 3rd party data structures to/from json
//! \note This is not for data structures internal to this project (those functions are in their respective
//! headers/sources)

//! \see https://json.nlohmann.me/features/arbitrary_types/
namespace nlohmann
{
    /**!
     * Serialization to/Deserialization from json for unique_ptr's
     */
    template <typename T>
    struct adl_serializer<std::unique_ptr<T>>
    {
        //! Deserialize from json
        static void from_json(const json& j, std::unique_ptr<T>& opt)
        {
            if(j.is_null())
            {
                opt = nullptr;
            }
            else
            {
                opt = std::make_unique<T>();
                j.get_to<T>(*opt);
            }
        }

        //! Serialize to json
        static void to_json(json& j, const std::unique_ptr<T>& opt)
        {
            if(opt.get())
            {
                j = *opt;
            }
            else
            {
                j = nullptr;
            }
        }
    };

    /**!
     * Serialization to/Deserialization from json for shared_ptr's
     */
    template <typename T>
    struct adl_serializer<std::shared_ptr<T>>
    {
        //! Deserialize from json
        static void from_json(const json& j, std::shared_ptr<T>& opt)
        {
            if(j.is_null())
            {
                opt = nullptr;
            }
            else
            {
                opt = std::make_shared<T>();
                j.get_to<T>(*opt);
            }
        }

        //! Serialize to json
        static void to_json(json& j, const std::shared_ptr<T>& opt)
        {
            if(opt.get())
            {
                j = *opt;
            }
            else
            {
                j = nullptr;
            }
        }
    };

    /**!
     * Serialization to/Deserialization from json for optional's
     */
    template <typename T>
    struct adl_serializer<std::optional<T>>
    {
        //! Deserialize from json
        static void from_json(const json& j, std::optional<T>& opt)
        {
            if(j.is_null())
            {
                opt = std::nullopt;
            }
            else
            {
                j.get_to<T>(*opt);
            }
        }

        //! Serialize to json
        static void to_json(json& j, const std::optional<T>& opt)
        {
            if(opt == std::nullopt)
            {
                j = nullptr;
            }
            else
            {
                j = *opt;
            }
        }
    };

    /**!
     * Serialization to/Deserialization from json for matrices and vectors
     */
    template <typename Derived>
    struct adl_serializer<Eigen::MatrixBase<Derived>>
    {
        //! Deserialize from json
        static void from_json(const json& j, Eigen::MatrixBase<Derived>& m)
        {
            using Scalar = typename Eigen::MatrixBase<Derived>::Scalar;

            for(std::size_t row = 0, num_rows = j.size(); row < num_rows; ++row)
            {
                const auto& json_row = j.at(row);
                for(std::size_t col = 0, num_cols = json_row.size(); col < num_cols; ++col)
                {
                    const auto& value = json_row.at(col);
                    m(row, col)       = value.get<Scalar>();
                }
            }
        }

        //! Serialize to json
        static void to_json(json& j, const Eigen::MatrixBase<Derived>& m)
        {
            for(unsigned int row = 0, num_rows = m.rows(); row < num_rows; ++row)
            {
                nlohmann::json column = nlohmann::json::array();
                for(unsigned int col = 0, num_cols = m.cols(); col < num_cols; ++col)
                {
                    column.push_back(m(row, col));
                }
                j.push_back(column);
            }
        }
    };

    /**!
     * Serialization to/Deserialization from json for matrices and vectors
     */
    template <typename _Scalar, int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols>
    struct adl_serializer<Eigen::Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols>>
    {
        //! Deserialize from json
        static void from_json(const json& j, Eigen::Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols>& m)
        {
            const unsigned int num_rows = m.rows() > 0 ? m.rows() : j.size();
            if(num_rows == 0)
            {
                return;
            }

            if(j[0].is_array())
            {
                const unsigned int num_cols = m.cols() > 0 ? m.cols() : j[0].size();
                m.resize(num_rows, num_cols);

                for(std::size_t row = 0; row < num_rows; ++row)
                {
                    const json& json_row = j.at(row);
                    for(std::size_t col = 0; col < num_cols; ++col)
                    {
                        const json& value = json_row.at(col);
                        m(row, col)       = value.get<_Scalar>();
                    }
                }
            }
            else if(j[0].is_number())
            {
                m.resize(num_rows, 1);
                for(std::size_t row = 0; row < num_rows; ++row)
                {
                    m(row, 0) = j.at(row).get<_Scalar>();
                }
            }
            else
            {
                throw grstapse::createLogicError("malformed json for eigen vector or matrix");
            }
        }

        //! Serialize to json
        static void to_json(json& j, const Eigen::Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols>& m)
        {
            const unsigned int num_rows = m.rows();
            const unsigned int num_cols = m.cols();

            j = nlohmann::json::array();
            if(num_cols > 1 && num_rows > 1)
            {
                for(unsigned int row = 0; row < num_rows; ++row)
                {
                    nlohmann::json column = nlohmann::json::array();
                    for(unsigned int col = 0; col < num_cols; ++col)
                    {
                        column.push_back(m(row, col));
                    }
                    j.push_back(column);
                }
            }
            else if(num_cols == 1)
            {
                for(unsigned int row = 0; row < num_rows; ++row)
                {
                    j.push_back(m(row, 0));
                }
            }
            else if(num_rows == 1)
            {
                for(unsigned int col = 0; col < num_cols; ++col)
                {
                    j.push_back(m(0, col));
                }
            }
            else
            {
                throw grstapse::createLogicError("Don't know how we got here...");
            }
        }
    };

    /**!
     * Serialization to/Deserialization from json for quaternions
     */
    template <typename Derived>
    struct adl_serializer<Eigen::QuaternionBase<Derived>>
    {
        //! Deserialize from json
        static void from_json(const json& j, Eigen::QuaternionBase<Derived>& q)
        {
            j[grstapse::constants::k_qw] = q.w();
            j[grstapse::constants::k_qx] = q.x();
            j[grstapse::constants::k_qy] = q.y();
            j[grstapse::constants::k_qz] = q.z();
        }

        //! Serialize to json
        static void to_json(json& j, const Eigen::QuaternionBase<Derived>& q)
        {
            using Scalar = typename Eigen::QuaternionBase<Derived>::Scalar;
            j.at(grstapse::constants::k_qw).get_to<Scalar>(q.w());
            j.at(grstapse::constants::k_qx).get_to<Scalar>(q.x());
            j.at(grstapse::constants::k_qy).get_to<Scalar>(q.y());
            j.at(grstapse::constants::k_qz).get_to<Scalar>(q.z());
        }
    };

    template <>
    struct adl_serializer<ompl::base::RealVectorBounds>
    {
        //! Deserialize from json
        static ompl::base::RealVectorBounds from_json(const json& j);
        //! Serialize to json
        static void to_json(json& j, const ompl::base::RealVectorBounds& b);
    };

    /**!
     * Serialization to/Deserialization from json for states in a SE2 state space
     */
    template <>
    struct adl_serializer<ompl::base::SE2StateSpace::StateType>
    {
        //! Deserialize from json
        static void from_json(const json& j, ompl::base::SE2StateSpace::StateType& s);
        //! Serialize to json
        static void to_json(json& j, const ompl::base::SE2StateSpace::StateType& s);
    };

    /**!
     * Serialization to/Deserialization from json for a SE2 state space
     */
    template <>
    struct adl_serializer<ompl::base::SE2StateSpace>
    {
        //! Deserialize from json
        static void from_json(const json& j, ompl::base::SE2StateSpace& s);
        //! Serialize to json
        static void to_json(json& j, const ompl::base::SE2StateSpace& s);
    };

    /**!
     * Serialization to/Deserialization from json for states in a SE3 state space
     */
    template <>
    struct adl_serializer<ompl::base::SE3StateSpace::StateType>
    {
        //! Deserialize from json
        static void from_json(const json& j, ompl::base::SE3StateSpace::StateType& s);
        //! Serialize to json
        static void to_json(json& j, const ompl::base::SE3StateSpace::StateType& s);
    };

    /**!
     * Serialization to/Deserialization from json for a SE3 state space
     */
    template <>
    struct adl_serializer<ompl::base::SE3StateSpace>
    {
        //! Deserialize from json
        static void from_json(const json& j, ompl::base::SE3StateSpace& s);
        //! Serialize to json
        static void to_json(json& j, const ompl::base::SE3StateSpace& s);
    };

    /**!
     * Serialization to/Deserialization from json for states in a SO2 state space
     */
    template <>
    struct adl_serializer<ompl::base::SO3StateSpace::StateType>
    {
        //! Deserialize from json
        static void from_json(const json& j, ompl::base::SO3StateSpace::StateType& s);
        //! Serialize to json
        static void to_json(json& j, const ompl::base::SO3StateSpace::StateType& s);
    };

    /**!
     * Serialization to/Deserialization from json for a path
     */
    template <>
    struct adl_serializer<ompl::geometric::PathGeometric>
    {
        //! Deserialize from json
        //        static void from_json(const json& j, ompl::geometric::PathGeometric& p);
        //! Serialize to json
        static void to_json(json& j, const ompl::geometric::PathGeometric& p);
    };
}  // namespace nlohmann