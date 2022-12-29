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
#include "grstapse/task_allocation/itags/vector_reduction_function.hpp"

// Local
#include "grstapse/common/utilities/constants.hpp"
#include "grstapse/common/utilities/custom_json_conversions.hpp"
#include "grstapse/common/utilities/error.hpp"
#include "grstapse/task_allocation/itags/threshold_cummulative_function.hpp"

namespace grstapse
{
    std::shared_ptr<VectorReductionFunction> VectorReductionFunction::load(const nlohmann::json& j)
    {
        VectorReductionFunctionType type =
            j.at(constants::k_vector_reduction_function_type).get<VectorReductionFunctionType>();
        switch(type)
        {
            case VectorReductionFunctionType::e_threshold_cumulative:
                return j.get<std::shared_ptr<ThresholdCumulativeFunction>>();
            default:
                throw createLogicError("Unknown vector reduction function type");
        }
    }
}  // namespace grstapse