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

// Local
#include "grstapse/task_allocation/itags/vector_reduction_function.hpp"

namespace grstapse
{
    /**!
     * Function determines how many elements meet or surpass a threshold
     */
    class ThresholdCumulativeFunction : public VectorReductionFunction
    {
       public:
        //! For json
        ThresholdCumulativeFunction() = default;

        //! Constructor
        explicit ThresholdCumulativeFunction(const float threshold);

        //! \copydoc VectorReductionFunction
        float reduce(const Eigen::VectorXf& v) const final override;

       private:
        float m_threshold;

        friend void from_json(const nlohmann::json& j, ThresholdCumulativeFunction& f);
    };

    void from_json(const nlohmann::json& j, ThresholdCumulativeFunction& f);

}  // namespace grstapse