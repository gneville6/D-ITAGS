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
#include "grstapse/geometric_planning/ompl/ompl_motion_planner_parameters.hpp"

// Local
#include "grstapse/common/utilities/constants.hpp"
#include "grstapse/common/utilities/json_field_validator.hpp"

namespace grstapse
{
    OmplMotionPlannerParameters::OmplMotionPlannerParameters()
        : MotionPlannerParametersBase()
        , simplify_path(false)
        , simplify_path_timeout(-1.0f)
        , connection_range(0.1f)
        , solutions_window(10)
        , convergence_epislon(0.1)
    {}

    OmplMotionPlannerParameters::OmplMotionPlannerParameters(const float timeout,
                                                             const bool simplify_path,
                                                             const float simplify_path_timeout,
                                                             const float connection_range,
                                                             const unsigned int solutions_window,
                                                             const float convergence_epislon)
        : MotionPlannerParametersBase(timeout)
        , simplify_path(simplify_path)
        , simplify_path_timeout(simplify_path_timeout)
        , connection_range(connection_range)
        , solutions_window(solutions_window)
        , convergence_epislon(convergence_epislon)
    {}

    std::shared_ptr<const OmplMotionPlannerParameters> OmplMotionPlannerParameters::loadJson(const nlohmann::json& j)
    {
        validate(j,
                 {{constants::k_configuration_type, nlohmann::json::value_t::string},
                  {constants::k_timeout, nlohmann::json::value_t::number_float},
                  {constants::k_simplify_path, nlohmann::json::value_t::boolean},
                  {constants::k_simplify_path_timeout, nlohmann::json::value_t::number_float},
                  {constants::k_connection_range, nlohmann::json::value_t::number_float}});
        auto rv = std::make_shared<OmplMotionPlannerParameters>();
        j.at(constants::k_simplify_path).get_to(rv->simplify_path);
        j.at(constants::k_simplify_path_timeout).get_to(rv->simplify_path_timeout);
        j.at(constants::k_connection_range).get_to(rv->connection_range);
        if(j.contains(constants::k_solutions_window))
        {
            j.at(constants::k_solutions_window).get_to(rv->solutions_window);
        }
        if(j.contains(constants::k_convergence_epislon))
        {
            j.at(constants::k_convergence_epislon).get_to(rv->convergence_epislon);
        }
        rv->internalLoadJson(j);
        return rv;
    }
}  // namespace grstapse