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
#include "grstapse/geometric_planning/motion_planner_parameters_base.hpp"

// Local
#include "grstapse/common/utilities/constants.hpp"
#include "grstapse/common/utilities/error.hpp"
#include "grstapse/common/utilities/json_field_validator.hpp"
#include "grstapse/geometric_planning/configuration_base.hpp"
#include "grstapse/geometric_planning/ompl/ompl_motion_planner_parameters.hpp"

namespace grstapse
{
    MotionPlannerParametersBase::MotionPlannerParametersBase()
        : timeout(-1.0f)
    {}

    MotionPlannerParametersBase::MotionPlannerParametersBase(const float timeout)
        : timeout(timeout)
    {}

    std::shared_ptr<const MotionPlannerParametersBase> MotionPlannerParametersBase::loadJson(const nlohmann::json& j)
    {
        validate(j, {{constants::k_configuration_type, nlohmann::json::value_t::string}});
        const ConfigurationType configuration_type = j.at(constants::k_configuration_type).get<ConfigurationType>();

        std::shared_ptr<const MotionPlannerParametersBase> rv;
        switch(configuration_type)
        {
            case ConfigurationType::e_ompl:
            {
                return OmplMotionPlannerParameters::loadJson(j);
            }
            case ConfigurationType::e_graph:
            {
                throw createLogicError("Not Implemented");
            }
            default:
            {
                throw createLogicError("Unknown configuration type");
            }
        }
    }
    void MotionPlannerParametersBase::internalLoadJson(const nlohmann::json& j)
    {
        validate(j,
                 {{constants::k_configuration_type, nlohmann::json::value_t::string},
                  {constants::k_timeout, nlohmann::json::value_t::number_float}});
        j.at(constants::k_configuration_type).get_to(configuration_type);
        j.at(constants::k_timeout).get_to(timeout);
    }

    // \note Needed to make this class polymorphic
    MotionPlannerParametersBase::~MotionPlannerParametersBase() = default;
}  // namespace grstapse