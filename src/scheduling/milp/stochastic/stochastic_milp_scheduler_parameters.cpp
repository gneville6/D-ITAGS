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
#include "grstapse/scheduling/milp/stochastic/stochastic_milp_scheduler_parameters.hpp"

// Local
#include "grstapse/common/utilities/constants.hpp"

namespace grstapse
{
    std::shared_ptr<const StochasticMilpSchedulerParameters> StochasticMilpSchedulerParameters::deserializeFromJson(
        const nlohmann::json& j)
    {
        auto rv = std::make_shared<StochasticMilpSchedulerParameters>();
        rv->MilpSchedulerParameters::internalDeserialize(j);
        j[constants::k_alpha].get_to(rv->alpha);
        j[constants::k_num_scenarios].get_to(rv->num_scenarios);
        return rv;
    }
}  // namespace grstapse