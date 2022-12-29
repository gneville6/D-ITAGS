

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

namespace grstapse
{
    //! The type of scheduling algorithm
    enum class SchedulerType : uint8_t
    {
        e_unknown = 0,
        e_milp
    };
    NLOHMANN_JSON_SERIALIZE_ENUM(SchedulerType,
                                 {{SchedulerType::e_unknown, "unknown"}, {SchedulerType::e_milp, "milp"}});

    /**!
     * Base class for the parameters for scheduling algorithms
     */
    struct SchedulerParameters
    {
        //! Load parameters for a scheduling algorithm (The type of scheduling algorithm is encoded in the json)
        static std::shared_ptr<const SchedulerParameters> deserializeFromJson(const nlohmann::json& j);

        //! Virtual destructor for polymorphism
        virtual ~SchedulerParameters() = default;

        SchedulerType scheduler_type;

       protected:
        //! Constructor
        explicit SchedulerParameters(SchedulerType scheduler_type);
    };
}  // namespace grstapse