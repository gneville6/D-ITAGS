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
#include "se3_ompl_configuration.hpp"

namespace grstapse
{
    /**!
     * Container for an SE(3) state that represents the initial/terminal configuration for a task
     */
    class Se3StateOmplConfiguration : public Se3OmplConfiguration
    {
       public:
        //! Deserializes from json
        static std::shared_ptr<const Se3StateOmplConfiguration> deserializeFromJson(const nlohmann::json& j);

        //! Default Constructor
        Se3StateOmplConfiguration();

        /**!
         * Constructor
         *
         * \param x
         * \param y
         * \param z
         */
        Se3StateOmplConfiguration(const float x, const float y, const float z);

        /**!
         * Constructor
         *
         * \param x
         * \param y
         * \param z
         * \param qw
         * \param qx
         * \param qy
         * \param qz
         */
        Se3StateOmplConfiguration(const float x,
                                  const float y,
                                  const float z,
                                  const float qw,
                                  const float qx,
                                  const float qy,
                                  const float qz);

        [[nodiscard]] inline float x() const;

        [[nodiscard]] inline float y() const;

        [[nodiscard]] inline float z() const;

        [[nodiscard]] inline float qw() const;

        [[nodiscard]] inline float qx() const;

        [[nodiscard]] inline float qy() const;

        [[nodiscard]] inline float qz() const;

        //! Equality
        [[nodiscard]] bool operator==(const Se3StateOmplConfiguration& rhs) const;

        //! \returns Whether \rhs is equal to this TaskConfigurationBase
        [[nodiscard]] virtual bool isEqual(const std::shared_ptr<const ConfigurationBase>& rhs) const final override;

        //! \copydoc OmplConfiguration
        [[nodiscard]] ompl::base::ScopedStatePtr convertToScopedStatePtr(
            const ompl::base::StateSpacePtr& state_space) const final override;
        //! \copydoc OmplConfiguration
        [[nodiscard]] ompl::base::GoalPtr convertToGoalPtr(
            const ompl::base::SpaceInformationPtr& space_information) const final override;

       private:
        //! \copydoc Se3OmplConfiguration
        [[nodiscard]] float euclideanDistance(
            const std::shared_ptr<const Se3StateOmplConfiguration>& rhs) const final override;

        float m_x;
        float m_y;
        float m_z;
        float m_qw;
        float m_qx;
        float m_qy;
        float m_qz;
    };

    float Se3StateOmplConfiguration::x() const
    {
        return m_x;
    }
    float Se3StateOmplConfiguration::y() const
    {
        return m_y;
    }
    float Se3StateOmplConfiguration::z() const
    {
        return m_z;
    }
    float Se3StateOmplConfiguration::qw() const
    {
        return m_qw;
    }
    float Se3StateOmplConfiguration::qx() const
    {
        return m_qx;
    }
    float Se3StateOmplConfiguration::qy() const
    {
        return m_qy;
    }
    float Se3StateOmplConfiguration::qz() const
    {
        return m_qz;
    }
}  // namespace grstapse