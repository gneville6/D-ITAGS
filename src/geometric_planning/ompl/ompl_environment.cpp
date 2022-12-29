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
#include "grstapse/geometric_planning/ompl/ompl_environment.hpp"

// External
#include <fmt/format.h>
// Local
#include "grstapse/common/utilities/constants.hpp"
#include "grstapse/common/utilities/custom_json_conversions.hpp"
#include "grstapse/common/utilities/error.hpp"
#include "grstapse/common/utilities/json_field_validator.hpp"
#include "grstapse/geometric_planning/pgm_environment.hpp"

namespace grstapse
{
    OmplEnvironment::OmplEnvironment(OmplEnvironmentType environment_type, OmplStateSpaceType state_space_type)
        : EnvironmentBase(ConfigurationType::e_ompl)
        , ompl::base::StateValidityChecker(nullptr)
        , m_state_space(nullptr)
        , m_environment_type(environment_type)
        , m_state_space_type(state_space_type)
    {}

    std::shared_ptr<OmplEnvironment> OmplEnvironment::deserializeFromJson(const nlohmann::json& j)
    {
        validate(j, {{constants::k_environment_type, nlohmann::json::value_t::string}});
        const OmplEnvironmentType environment_type = j.at(constants::k_environment_type).get<OmplEnvironmentType>();
        switch(environment_type)
        {
            case OmplEnvironmentType::e_pgm:
            {
                return j.get<std::shared_ptr<PgmEnvironment>>();
            }
            default:
            {
                throw createLogicError(fmt::format("Unknown environment type: {0:s}",
                                                   j.at(constants::k_environment_type).get<std::string>()));
            }
        }
    }
}  // namespace grstapse