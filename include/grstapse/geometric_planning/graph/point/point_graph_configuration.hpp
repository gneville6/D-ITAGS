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

// Local
#include "grstapse/geometric_planning/graph/graph_configuration.hpp"

namespace grstapse
{
    /**!
     * Configuration from a graph that represents a 2D point
     */
    class PointGraphConfiguration : public GraphConfiguration
    {
       public:
        //! Deserializes from json
        static std::shared_ptr<const PointGraphConfiguration> deserializeFromJson(const nlohmann::json& j);

        /**!
         * Constructor
         *
         * \param id The identifier for the graph vertex
         * \param x The x coordinate of this point
         * \param y The y coordinate of this point
         */
        PointGraphConfiguration(unsigned int id, float x, float y);

        //! \returns The x coordinate of this point
        [[nodiscard]] inline float x() const;

        //! \returns The y coordinate of this point
        [[nodiscard]] inline float y() const;

        //! \returns The euclidean distance to \p rhs
        [[nodiscard]] inline float euclideanDistance(const std::shared_ptr<PointGraphConfiguration>& rhs) const;

        //! \returns The euclidean distance to \p rhs
        [[nodiscard]] float euclideanDistance(const PointGraphConfiguration& rhs) const;

        //! \copydoc ConfigurationBase
        float euclideanDistance(const std::shared_ptr<const ConfigurationBase>& rhs) const final override;

        //! \copydoc ConfigurationBase
        bool isEqual(const std::shared_ptr<const ConfigurationBase>& rhs) const final override;

        //! Equality operator
        bool operator==(const PointGraphConfiguration& rhs) const;

       private:
        float m_x;
        float m_y;
    };

    // Inline Function
    float PointGraphConfiguration::x() const
    {
        return m_x;
    }

    float PointGraphConfiguration::y() const
    {
        return m_y;
    }

    float PointGraphConfiguration::euclideanDistance(const std::shared_ptr<PointGraphConfiguration>& rhs) const
    {
        return euclideanDistance(*rhs);
    }
}  // namespace grstapse