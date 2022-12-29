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
#include "grstapse/species.hpp"

// External
#include <magic_enum/magic_enum.hpp>
#include <nlohmann/json.hpp>
#include <ompl/geometric/PathGeometric.h>
// Local
#include "grstapse/common/utilities/custom_json_conversions.hpp"
#include "grstapse/common/utilities/json_field_validator.hpp"
#include "grstapse/geometric_planning/ompl/ompl_motion_planner.hpp"

namespace grstapse
{
    unsigned int Species::s_next_id = 0;

    Species::Species()
        : m_id(s_next_id++)
    {}

    Species::Species(const std::string& name,
                     const Eigen::VectorXf& traits,
                     const float radius,
                     const float speed,
                     const std::shared_ptr<MotionPlannerBase>& motion_planner)
        : m_id(s_next_id++)
        , m_name(name)
        , m_traits(traits)
        , m_bounding_radius(radius)
        , m_speed(speed)
        , m_motion_planner(motion_planner)
    {}

    std::shared_ptr<const Species> Species::loadJson(
        const nlohmann::json& j,
        const std::vector<std::shared_ptr<MotionPlannerBase>>& motion_planners)
    {
        validate(j,
                 {{constants::k_name, nlohmann::json::value_t::string},
                  {constants::k_traits, nlohmann::json::value_t::array},
                  {constants::k_bounding_radius, nlohmann::json::value_t::number_float},
                  {constants::k_speed, nlohmann::json::value_t::number_float},
                  {constants::k_mp_index, nlohmann::json::value_t::number_unsigned}});
        std::shared_ptr<Species> s = std::make_shared<Species>();
        j.at(constants::k_name).get_to<std::string>(s->m_name);
        j.at(constants::k_traits).get_to<Eigen::VectorXf>(s->m_traits);
        j.at(constants::k_bounding_radius).get_to<float>(s->m_bounding_radius);
        j.at(constants::k_speed).get_to<float>(s->m_speed);

        unsigned int mp_index;
        j.at(constants::k_mp_index).get_to<unsigned int>(mp_index);
        assert(mp_index < motion_planners.size());
        s->m_motion_planner = motion_planners[mp_index];

        return s;
    }
}  // namespace grstapse