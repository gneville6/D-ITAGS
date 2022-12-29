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

// Global
#include <string>
// External
#include <eigen3/Eigen/Core>
#include <nlohmann/json.hpp>
// Local
#include "grstapse/geometric_planning/ompl/ompl_enums.hpp"

namespace grstapse
{
    // Forward Declarations
    class MotionPlannerBase;
    class ConfigurationBase;
    class MotionPlannerQueryResultBase;

    /**!
     * A container for the information associated with a species of robots
     */
    class Species
    {
       public:
        //! Default constructor for json
        Species();

        //! Constructor
        Species(const std::string& name,
                const Eigen::VectorXf& traits,
                const float radius,
                const float speed,
                const std::shared_ptr<MotionPlannerBase>& motion_planner);

        //! \returns The name of the Species
        [[nodiscard]] inline const std::string& name() const;

        //! \returns An identifier for the Species
        [[nodiscard]] inline unsigned int id() const;

        //! \returns The traits for the species
        [[nodiscard]] inline const Eigen::VectorXf& traits() const;

        //! \returns The radius of a bounding circle/sphere for this species of robot
        [[nodiscard]] inline float boundingRadius() const;

        //! \returns The speed for this type of robot
        [[nodiscard]] inline float speed() const;

        //! \returns The the motion planner for this species of robot
        [[nodiscard]] inline const std::shared_ptr<MotionPlannerBase>& motionPlanner() const;

        /**!
         *  Deserializes a json object with species information
         *
         *  \note A custom function because the motion planners are needed
         */
        static std::shared_ptr<const Species> loadJson(
            const nlohmann::json& j,
            const std::vector<std::shared_ptr<MotionPlannerBase>>& motion_planners);

       private:
        unsigned int m_id;
        std::string m_name;
        Eigen::VectorXf m_traits;
        float m_bounding_radius;
        float m_speed;
        std::shared_ptr<MotionPlannerBase> m_motion_planner;

        static unsigned int s_next_id;
    };

    // Inline Functions
    const std::string& Species::name() const
    {
        return m_name;
    }

    unsigned int Species::id() const
    {
        return m_id;
    }

    const Eigen::VectorXf& Species::traits() const
    {
        return m_traits;
    }

    float Species::boundingRadius() const
    {
        return m_bounding_radius;
    }

    float Species::speed() const
    {
        return m_speed;
    }

    const std::shared_ptr<MotionPlannerBase>& Species::motionPlanner() const
    {
        return m_motion_planner;
    }
}  // namespace grstapse