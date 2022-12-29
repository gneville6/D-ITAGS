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

// External
#include <nlohmann/json.hpp>

namespace grstapse
{
    //! An enumeration of possible statuses resulting from a motion planning query
    enum class MotionPlannerQueryStatus : uint8_t
    {
        e_unknown = 0,
        e_success,
        e_timeout
    };
    NLOHMANN_JSON_SERIALIZE_ENUM(MotionPlannerQueryStatus,
                                 {{MotionPlannerQueryStatus::e_unknown, "unknown"},
                                  {MotionPlannerQueryStatus::e_success, "success"},
                                  {MotionPlannerQueryStatus::e_timeout, "timeout"}});

    /**!
     * Abstract base class for the result of a motion planning query
     */
    class MotionPlanningQueryResultBase
    {
       public:
        //! \returns The status of the motion plan query
        [[nodiscard]] inline MotionPlannerQueryStatus status() const;

        //! \returns The amount of time needed to execution the motion plan assuming constant \p speed
        [[nodiscard]] inline virtual float duration(const float speed) const;

        //! \returns The length of the motion plan
        [[nodiscard]] virtual float length() const = 0;

        //! Serialize the motion planning result to json
        virtual void serializeToJson(nlohmann::json& j) const = 0;

       protected:
        //! Default constructor
        explicit MotionPlanningQueryResultBase(MotionPlannerQueryStatus status);

        MotionPlannerQueryStatus m_status;
    };

    /**!
     * Concept to force a type to derive from MotionPlanningQueryResultBase
     *
     * \tparam T Derivative of MotionPlanningQueryResultBase
     */
    template <typename T>
    concept MotionPlanningQueryResultDeriv = std::derived_from<T, MotionPlanningQueryResultBase>;

    // Inline Functions
    MotionPlannerQueryStatus MotionPlanningQueryResultBase::status() const
    {
        return m_status;
    }

    float MotionPlanningQueryResultBase::duration(const float speed) const
    {
        return length() / speed;
    }
}  // namespace grstapse