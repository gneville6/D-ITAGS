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
// Local
#include "grstapse/scheduling/scheduler_parameters.hpp"

namespace grstapse
{
    //! Type of MILP formulation algorithm
    enum class MilpSchedulerType : uint8_t
    {
        e_base,
        e_deterministic,
        e_stochastic
    };
    NLOHMANN_JSON_SERIALIZE_ENUM(MilpSchedulerType,
                                 {{MilpSchedulerType::e_base, "base"},
                                  {MilpSchedulerType::e_deterministic, "deterministic"},
                                  {MilpSchedulerType::e_stochastic, "stochastic"}})

    /**!
     * Container for the parameters used by the MILP solver for a scheduling problem
     */
    struct MilpSchedulerParameters : public SchedulerParameters
    {
        //! Default Constructor
        MilpSchedulerParameters();

        //!
        static std::shared_ptr<const MilpSchedulerParameters> deserializeFromJson(const nlohmann::json& j);

        MilpSchedulerType milp_scheduler_type;
        float timeout;
        unsigned int threads;
        bool compute_transition_duration_heuristic;

       protected:
        void internalDeserialize(const nlohmann::json& j);
    };

}  // namespace grstapse