/*
 * Graphically Recursive Simultaneous Task Allocation, Planning,
 * Scheduling, and Execution
 *
 * Copyright (C) 2020-2022
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
#include <nlohmann/json.hpp>
// Local
#include "grstapse/geometric_planning/configuration_base.hpp"

namespace grstapse
{
    /**!
     * Parameters for motion planning
     */
    struct MotionPlannerParametersBase
    {
        //! Default Constructor
        MotionPlannerParametersBase();

        /**!
         * Destructor
         *
         * \note Needed to make this class polymorphic
         */
        virtual ~MotionPlannerParametersBase();

        //! Constructor
        explicit MotionPlannerParametersBase(const float timeout);

        //! Deserializes from json
        static std::shared_ptr<const MotionPlannerParametersBase> loadJson(const nlohmann::json& j);

        ConfigurationType configuration_type;
        float timeout;

       protected:
        void internalLoadJson(const nlohmann::json& j);
    };

    /**!
     * Concept to force a type to derive from MotionPlannerParametersBase
     *
     * \tparam T Derivative of MotionPlannerParametersBase
     */
    template <typename T>
    concept MotionPlannerParametersDeriv = std::derived_from<T, MotionPlannerParametersBase>;
}  // namespace grstapse