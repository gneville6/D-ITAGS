/*
 * Graphically Recursive Simultaneous Task Allocation, Planning,
 * Scheduling, and Execution
 *
 * Copyright (C) 2020-2021
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

// Global
#include <memory>
#include <string>

namespace grstapse
{
    // Forward Declaration
    class Species;
    class ConfigurationBase;
    class MotionPlanningQueryResultBase;

    /**!
     * A container for the information about a single robot
     */
    class Robot
    {
       public:
        //! Constructor
        Robot(const std::string& name,
              const std::shared_ptr<const ConfigurationBase>& initial_configuration,
              const std ::shared_ptr<const Species>& species);

        //! \returns The name of the robot
        [[nodiscard]] inline const std::string& name() const;

        //! \returns An identifier for the robot
        [[nodiscard]] inline unsigned int id() const;

        //! \returns The initial configuration of the robot
        [[nodiscard]] inline const std::shared_ptr<const ConfigurationBase>& initialConfiguration() const;

        //! \returns The species of the robot
        [[nodiscard]] inline const std::shared_ptr<const Species>& species() const;

        //! \returns The radius of a bounding circle/sphere for this robot
        [[nodiscard]] float boundingRadius() const;

        //! \returns The speed for this robot
        [[nodiscard]] float speed() const;

        /**!
         * \returns The result of a motion planning query for this robot from \p initial to \p terminal
         */
        [[nodiscard]] std::shared_ptr<const MotionPlanningQueryResultBase> motionPlanningQuery(
            const std::shared_ptr<const ConfigurationBase>& initial,
            const std::shared_ptr<const ConfigurationBase>& terminal) const;

        /**!
         * \returns The result of a motion planning query for this robot from its initial
         *          configuration to \p terminal
         */
        [[nodiscard]] inline std::shared_ptr<const MotionPlanningQueryResultBase> motionPlanningQuery(
            const std::shared_ptr<const ConfigurationBase>& terminal) const;

        /**!
         * \returns The duration of a motion plan for this robot from \p initial to \p terminal
         */
        [[nodiscard]] float durationQuery(const std::shared_ptr<const ConfigurationBase>& initial,
                                          const std::shared_ptr<const ConfigurationBase>& terminal) const;

        /**!
         * \returns The duration of a motion plan for this robot its initial
         *          configuration to \p terminal
         */
        [[nodiscard]] inline float durationQuery(const std::shared_ptr<const ConfigurationBase>& terminal) const;

        /**!
         * \returns Whether the query for a motion plan from \p initial to \p terminal for a robot with the same
         *          bounding radius as this robot has been computed
         */
        [[nodiscard]] bool isMemoized(const std::shared_ptr<const ConfigurationBase>& initial,
                                      const std::shared_ptr<const ConfigurationBase>& terminal) const;

        /**!
         * \returns Whether the query for a motion plan from \p initial to \p terminal for a robot with the same
         *          bounding radius as this species has been computed
         */
        [[nodiscard]] inline bool isMemoized(const std::shared_ptr<const ConfigurationBase>& terminal) const;

       private:
        std::shared_ptr<const ConfigurationBase> m_initial_configuration;
        std::string m_name;
        std::shared_ptr<const Species> m_species;

        unsigned int m_id;
        static unsigned int s_next_id;
    };

    // Inline functions
    const std::string& Robot::name() const
    {
        return m_name;
    }

    unsigned int Robot::id() const
    {
        return m_id;
    }

    const std::shared_ptr<const ConfigurationBase>& Robot::initialConfiguration() const
    {
        return m_initial_configuration;
    }

    const std::shared_ptr<const Species>& Robot::species() const
    {
        return m_species;
    }

    std::shared_ptr<const MotionPlanningQueryResultBase> Robot::motionPlanningQuery(
        const std::shared_ptr<const ConfigurationBase>& terminal) const
    {
        return motionPlanningQuery(m_initial_configuration, terminal);
    }

    float Robot::durationQuery(const std::shared_ptr<const ConfigurationBase>& terminal) const
    {
        return durationQuery(m_initial_configuration, terminal);
    }

    bool Robot::isMemoized(const std::shared_ptr<const ConfigurationBase>& terminal) const
    {
        return isMemoized(m_initial_configuration, terminal);
    }
}  // namespace grstapse