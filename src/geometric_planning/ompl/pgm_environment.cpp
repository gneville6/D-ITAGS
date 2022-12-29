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
#include "grstapse/geometric_planning/pgm_environment.hpp"

// External
#include <ompl/base/spaces/DubinsStateSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <yaml-cpp/yaml.h>
// Local
#include "grstapse/common/utilities/constants.hpp"
#include "grstapse/common/utilities/json_field_validator.hpp"
#include "grstapse/common/utilities/logger.hpp"
#include "grstapse/geometric_planning/ompl/ompl_enums.hpp"
#include "grstapse/species.hpp"

namespace grstapse
{
    PgmEnvironment::PgmEnvironment()
        : OmplEnvironment{.environment_type = OmplEnvironmentType::e_pgm, .state_space_type = OmplStateSpaceType::e_se2}
    {
        m_state_space = std::make_shared<ompl::base::SE2StateSpace>();
    }

    PgmEnvironment::PgmEnvironment(const std::string& filepath,
                                   const float resolution,
                                   const float origin_x,
                                   const float origin_y)
        : OmplEnvironment{.environment_type = OmplEnvironmentType::e_pgm, .state_space_type = OmplStateSpaceType::e_se2}
        , m_resolution(resolution)
        , m_origin_x(origin_x)
        , m_origin_y(origin_y)
    {
        m_pgm.loadFile(filepath.c_str());

        m_state_space = std::make_shared<ompl::base::SE2StateSpace>();
        ompl::base::RealVectorBounds bounds(2);
        bounds.low  = {minX(), minY()};
        bounds.high = {maxX(), maxY()};
        m_state_space->as<ompl::base::SE2StateSpace>()->setBounds(bounds);
    }

    bool PgmEnvironment::isValid(const ompl::base::State* state) const
    {
        const auto [cx, cy] = toCell(state->as<ompl::base::RealVectorStateSpace::StateType>()->values[0],
                                     state->as<ompl::base::RealVectorStateSpace::StateType>()->values[1]);

        const int cr = static_cast<int>(m_species->boundingRadius() / m_resolution);

        for(int x = cx - cr, xend = cx + cr; x <= xend; ++x)
        {
            // Outside the map
            if(x < 0 || x >= m_pgm.width())
            {
                continue;
            }

            for(int y = cy - cr, yend = cy + cr; y <= yend; ++y)
            {
                // Outside the map
                if(y < 0 || y >= m_pgm.height())
                {
                    continue;
                }

                // Outside the circle
                if(pow(x - cx, 2) + pow(y - cy, 2) >= pow(cr, 2))
                {
                    continue;
                }

                const unsigned int p = m_pgm.pixel(y, x);
                if(p < 127)
                {
                    return false;
                }
            }
        }
        return true;
    }

    float PgmEnvironment::longestPath() const
    {
        // Perimeter of map
        float longest_path = 2 * (maxY() - minY()) + 2 * (maxX() - minX());

        for(unsigned int x = 0, x_end = m_pgm.width(); x < x_end; ++x)
        {
            for(unsigned int y = 0, y_end = m_pgm.height(); y < y_end; ++y)
            {
                // Obstacle
                if (m_pgm.pixel(y, x) < 127) {
                    // Perimeter of cell
                    longest_path += m_resolution * 4;
                }
            }
        }

        return longest_path;
    }

    void from_json(const nlohmann::json& j, PgmEnvironment& e)
    {
        validate(j, {{constants::k_yaml_filepath, nlohmann::json::value_t::string}});

        const std::string yaml_filepath = j.at(constants::k_yaml_filepath);
        std::string image_filename;
        {
            YAML::Node doc    = YAML::LoadFile(yaml_filepath);
            image_filename    = doc[constants::k_image].as<std::string>();
            e.m_resolution    = doc[constants::k_resolution].as<float>();
            YAML::Node origin = doc[constants::k_origin];
            e.m_origin_x      = origin[0].as<float>();
            e.m_origin_y      = origin[1].as<float>();
        }
        const std::string pgm_filepath = yaml_filepath.substr(0, yaml_filepath.find_last_of('/') + 1) + image_filename;
        e.m_pgm.loadFile(pgm_filepath.c_str());

        if(auto j_itr = j.find(constants::k_dubins); j_itr != j.end() && (*j_itr).get<bool>())
        {
            validate(j, {{constants::k_turning_radius, nlohmann::json::value_t::number_float}});
            e.m_turning_radius = j.at(constants::k_turning_radius);
            e.m_state_space    = std::make_shared<ompl::base::DubinsStateSpace>(e.m_turning_radius);
        }
        else
        {
            e.m_state_space = std::make_shared<ompl::base::SE2StateSpace>();
        }

        ompl::base::RealVectorBounds bounds(2);
        bounds.low  = {e.minX(), e.minY()};
        bounds.high = {e.maxX(), e.maxY()};
        e.m_state_space->as<ompl::base::SE2StateSpace>()->setBounds(bounds);
    }
}  // namespace grstapse