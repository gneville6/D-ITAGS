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
#include "grstapse/robot.hpp"

// Local
#include "grstapse/geometric_planning/configuration_base.hpp"
#include "grstapse/geometric_planning/motion_planner_base.hpp"
#include "grstapse/species.hpp"

namespace grstapse
{
    unsigned int Robot::s_next_id = 0;

    Robot::Robot(const std::string& name,
                 const std::shared_ptr<const ConfigurationBase>& initial_configuration,
                 const std::shared_ptr<const Species>& species)
        : m_id(s_next_id++)
        , m_name(name)
        , m_initial_configuration(initial_configuration)
        , m_species(species)
    {}

    float Robot::boundingRadius() const
    {
        return m_species->boundingRadius();
    }

    float Robot::speed() const
    {
        return m_species->speed();
    }

    std::shared_ptr<const MotionPlanningQueryResultBase> Robot::motionPlanningQuery(
        const std::shared_ptr<const ConfigurationBase>& initial,
        const std::shared_ptr<const ConfigurationBase>& terminal) const
    {
        return m_species->motionPlanner()->query(m_species, initial, terminal);
    }

    float Robot::durationQuery(const std::shared_ptr<const ConfigurationBase>& initial,
                               const std::shared_ptr<const ConfigurationBase>& terminal) const
    {
        return m_species->motionPlanner()->durationQuery(m_species, initial, terminal);
    }

    bool Robot::isMemoized(const std::shared_ptr<const ConfigurationBase>& initial,
                           const std::shared_ptr<const ConfigurationBase>& terminal) const
    {
        return m_species->motionPlanner()->isMemoized(m_species, initial, terminal);
    }
}  // namespace grstapse