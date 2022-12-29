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

namespace grstapse
{
    enum class VectorReductionFunctionType
    {
        e_unknown,
        e_threshold_cumulative
    };
    NLOHMANN_JSON_SERIALIZE_ENUM(VectorReductionFunctionType,
                                 {{VectorReductionFunctionType::e_unknown, "unknown"},
                                  {VectorReductionFunctionType::e_threshold_cumulative, "threshold_cumulative"}});

    /**!
     * Base class for custom functions for reducing a vector to a single value
     *
     * \see RobotTraitsMatrixReduction
     */
    struct VectorReductionFunction
    {
        //! Deserializes from json
        static std::shared_ptr<VectorReductionFunction> load(const nlohmann::json& j);

        //! Reduce a vector to an single value
        virtual float reduce(const Eigen::VectorXf& v) const = 0;
    };

}  // namespace grstapse