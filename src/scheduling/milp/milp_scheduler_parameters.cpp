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
#include "grstapse/scheduling/milp/milp_scheduler_parameters.hpp"

// Local
#include "grstapse/common/utilities/constants.hpp"
#include "grstapse/common/utilities/error.hpp"
#include "grstapse/common/utilities/json_field_validator.hpp"
#include "grstapse/scheduling/milp/deterministic/deterministic_milp_scheduler_parameters.hpp"
#include "grstapse/scheduling/milp/stochastic/stochastic_milp_scheduler_parameters.hpp"

namespace grstapse
{
    MilpSchedulerParameters::MilpSchedulerParameters()
        : SchedulerParameters(SchedulerType::e_milp)
    {}

    std::shared_ptr<const MilpSchedulerParameters> MilpSchedulerParameters::deserializeFromJson(const nlohmann::json& j)
    {
        validate(j, {{constants::k_milp_scheduler_type, nlohmann::json::value_t::string}});
        const MilpSchedulerType type = j.at(constants::k_milp_scheduler_type).get<MilpSchedulerType>();
        switch(type)
        {
            case MilpSchedulerType::e_base:
            {
                auto rv = std::shared_ptr<MilpSchedulerParameters>(new MilpSchedulerParameters);
                rv->internalDeserialize(j);
                return rv;
            }
            case MilpSchedulerType::e_deterministic:
            {
                return DeterministicMilpSchedulerParameters::deserializeFromJson(j);
            }
            case MilpSchedulerType::e_stochastic:
            {
                return StochasticMilpSchedulerParameters::deserializeFromJson(j);
            }
            default:
            {
                throw createLogicError("Unknown scheduler type");
            }
        }
    }

    void MilpSchedulerParameters::internalDeserialize(const nlohmann::json& j)
    {
        j[constants::k_milp_scheduler_type].get_to(milp_scheduler_type);
        j[constants::k_timeout].get_to(timeout);
        j[constants::k_threads].get_to(threads);
        j[constants::k_compute_transition_duration_heuristic].get_to(compute_transition_duration_heuristic);
    }
}  // namespace grstapse