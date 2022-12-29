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

// External
#include <ompl/base/Goal.h>
#include <ompl/base/ScopedState.h>
// Local
#include "grstapse/geometric_planning/configuration_base.hpp"

#include "ompl_enums.hpp"

namespace grstapse
{
    /**!
     * A configuration used with an OMPL motion planner
     *
     * \see OmplMotionPlanner
     */
    class OmplConfiguration : public ConfigurationBase
    {
       public:
        //! Deserializes from json
        static std::shared_ptr<const OmplConfiguration> deserializeFromJson(const nlohmann::json& j);

        //! \returns
        [[nodiscard]] inline OmplGoalType goalType() const;

        //! \returns
        [[nodiscard]] inline OmplStateSpaceType stateSpaceType() const;

        //! \returns A scoped state ptr representation of this task configuration
        virtual ompl::base::ScopedStatePtr convertToScopedStatePtr(
            const ompl::base::StateSpacePtr& state_space) const = 0;

        //! \returns A goal ptr representation of this task configuration
        virtual ompl::base::GoalPtr convertToGoalPtr(
            const ompl::base::SpaceInformationPtr& space_information) const = 0;

       protected:
        /**!
         * Constructor
         *
         * \param goal_type
         * \param state_space_type
         */
        OmplConfiguration(OmplGoalType goal_type, OmplStateSpaceType state_space_type);

        //! Minimum euclidean distance from this task configuration base to \p rhs
        [[nodiscard]] virtual float euclideanDistance(const std::shared_ptr<const ConfigurationBase>& rhs) const = 0;

        OmplGoalType m_goal_type;
        OmplStateSpaceType m_state_space_type;
    };
    // Inline Functions
    OmplGoalType OmplConfiguration::goalType() const
    {
        return m_goal_type;
    }

    OmplStateSpaceType OmplConfiguration::stateSpaceType() const
    {
        return m_state_space_type;
    }
}  // namespace grstapse