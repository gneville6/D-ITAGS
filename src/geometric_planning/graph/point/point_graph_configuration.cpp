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
#include "grstapse/geometric_planning/graph/point/point_graph_configuration.hpp"

// Local
#include "grstapse/common/utilities/error.hpp"

namespace grstapse
{
    std::shared_ptr<const PointGraphConfiguration> PointGraphConfiguration::deserializeFromJson(const nlohmann::json& j)
    {
        // TODO(andrew)
        throw std::logic_error("Not implemented");
    }

    PointGraphConfiguration::PointGraphConfiguration(unsigned int id, float x, float y)
        : GraphConfiguration(id)
        , m_x(x)
        , m_y(y)
    {}

    float PointGraphConfiguration::euclideanDistance(const PointGraphConfiguration& rhs) const
    {
        // Note: split up because of some bug with sqrt
        const float x_diff    = static_cast<float>(m_x) - static_cast<float>(rhs.m_x);
        const float y_diff    = static_cast<float>(m_y) - static_cast<float>(rhs.m_y);
        const float x_diff_sq = powf(x_diff, 2.0f);
        const float y_diff_sq = powf(y_diff, 2.0f);
        const float sq_sum    = x_diff_sq + y_diff_sq;
        return std::sqrt(sq_sum);
    }

    float PointGraphConfiguration::euclideanDistance(const std::shared_ptr<const ConfigurationBase>& rhs) const
    {
        auto pgc_rhs = std::dynamic_pointer_cast<const PointGraphConfiguration>(rhs);
        if(!pgc_rhs)
        {
            throw createLogicError("rhs is not a 'PointGraphConfiguration'");
        }

        return euclideanDistance(*pgc_rhs);
    }

    bool PointGraphConfiguration::isEqual(const std::shared_ptr<const ConfigurationBase>& rhs) const
    {
        auto pgc_rhs = std::dynamic_pointer_cast<const PointGraphConfiguration>(rhs);
        if(!pgc_rhs)
        {
            return false;
        }

        return *this == *pgc_rhs;
    }

    bool PointGraphConfiguration::operator==(const PointGraphConfiguration& rhs) const
    {
        return m_id == rhs.m_id && m_x == rhs.m_x && m_y == rhs.m_y;
    }
}  // namespace grstapse