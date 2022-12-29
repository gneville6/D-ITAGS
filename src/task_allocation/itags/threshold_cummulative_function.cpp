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
#include "grstapse/task_allocation/itags/threshold_cummulative_function.hpp"

// Local
#include "grstapse/common/utilities/constants.hpp"

namespace grstapse
{
    ThresholdCumulativeFunction::ThresholdCumulativeFunction(const float threshold)
        : m_threshold(threshold)
    {}

    float ThresholdCumulativeFunction::reduce(const Eigen::VectorXf& v) const
    {
        return (v.array() > m_threshold).cast<float>().sum();
    }

    void from_json(const nlohmann::json& j, ThresholdCumulativeFunction& f)
    {
        j.at(constants::k_threshold).get_to(f.m_threshold);
    }
}  // namespace grstapse