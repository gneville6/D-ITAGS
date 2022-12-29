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
#include "grstapse/geometric_planning/graph/graph_motion_planner_base.hpp"

namespace grstapse
{
    // Forward Declarations
    class PointGraphConfiguration;
    class PointGraphMotionPlanner;
    class SampledPointGraphEnvironment;

    /**!
     *
     */
    class SampledPointGraphMotionPlanner : public GraphMotionPlannerBase
    {
       public:
        //! Constructor
        SampledPointGraphMotionPlanner(const std::shared_ptr<const MotionPlannerParametersBase>& parameters,
                                       const std::shared_ptr<SampledPointGraphEnvironment>& environment);

        /**!
         * Custom query for a path from \p initial_configuration to \p goal_configuration in the \p index'th graph
         *
         * \param species The species of the robot
         * \param initial_configuration The initial geometric configuration of the robot
         * \param goal_configuration The target geometric configuration of the robot
         *
         * \returns The result of the motion planning query
         */
        [[nodiscard]] std::shared_ptr<const MotionPlanningQueryResultBase> query(
            unsigned int index,
            const std::shared_ptr<const Species>& species,
            const std::shared_ptr<const PointGraphConfiguration>& initial_configuration,
            const std::shared_ptr<const PointGraphConfiguration>& goal_configuration);

        /**!
         * Checks if a path from \p start_state to \p goal_configuration has been memoized from the \p index'th graph
         *
         * \param species The species of the robot
         * \param initial_configuration The initial geometric configuration of the robot
         * \param goal_configuration The target geometric configuration of the robot
         *
         * \returns Whether a path from \p start_state to \p goal_state has been memoized
         */
        [[nodiscard]] bool isMemoized(unsigned int index,
                                      const std::shared_ptr<const Species>& species,
                                      const std::shared_ptr<const PointGraphConfiguration>& initial_configuration,
                                      const std::shared_ptr<const PointGraphConfiguration>& goal_configuration) const;

        /**!
         * Custom query for the duration to execute the path from \p initial_configuration to \p
         * goal_configuration in the \p index'th graph
         *
         * \param species The species of the robot
         * \param initial_configuration The initial geometric configuration of the robot
         * \param goal_configuration The target geometric configuration of the robot
         *
         * \returns The status of the planner and the path generated as the solution if possible
         */
        [[nodiscard]] float durationQuery(unsigned int index,
                                          const std::shared_ptr<const Species>& species,
                                          const std::shared_ptr<const PointGraphConfiguration>& initial_configuration,
                                          const std::shared_ptr<const PointGraphConfiguration>& goal_configuration);

       protected:
        //! \copydoc MotionPlannerBase
        [[nodiscard]] std::shared_ptr<const MotionPlanningQueryResultBase> computeMotionPlan(
            const std::shared_ptr<const Species>& species,
            const std::shared_ptr<const ConfigurationBase>& initial_configuration,
            const std::shared_ptr<const ConfigurationBase>& goal_configuration) override;

       private:
        // Hide base functions
        using MotionPlannerBase::durationQuery;
        using MotionPlannerBase::isMemoized;
        using MotionPlannerBase::query;

        std::vector<std::shared_ptr<PointGraphMotionPlanner>> m_sub_motion_planners;
    };

}  // namespace grstapse