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
#include "grstapse/scheduling/scheduler_parameters.hpp"

// Local
#include "grstapse/common/utilities/constants.hpp"
#include "grstapse/common/utilities/error.hpp"
#include "grstapse/scheduling/milp/milp_scheduler_parameters.hpp"
#include "grstapse/common/utilities/json_field_validator.hpp"

namespace grstapse
{
    std::shared_ptr<const SchedulerParameters> SchedulerParameters::deserializeFromJson(const nlohmann::json& j)
    {
        validate(j, {{constants::k_scheduler_type, nlohmann::json::value_t::string}});
        const SchedulerType type = j.at(constants::k_scheduler_type).get<SchedulerType>();
        switch(type)
        {
            case SchedulerType::e_milp:
            {
                return MilpSchedulerParameters::deserializeFromJson(j);
            }
            default:
            {
                throw createLogicError("Unknown scheduler type");
            }
        }
    }

    SchedulerParameters::SchedulerParameters(SchedulerType scheduler_type)
        : scheduler_type(scheduler_type)
    {}
}  // namespace grstapse