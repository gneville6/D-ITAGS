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

// Project
#include <grstapse/grstaps_problem_inputs.hpp>

namespace grstapse::mocks
{
    // Forward Declarations

    /**!
     * Mock to set some of the member variables for unit tests
     */
    class MockGrstapsProblemInputs : public GrstapsProblemInputs
    {
       public:
        //! Constructor
        MockGrstapsProblemInputs() = default;

        /**!
         * Sets the robot traits matrix reduction
         *
         * \note For AllocationPercentageRemaining tests
         */
        inline void setRobotTraitsMatrixReduction(
            const std::shared_ptr<const RobotTraitsMatrixReduction>& robot_traits_matrix_reduction)
        {
            m_robot_traits_matrix_reduction = robot_traits_matrix_reduction;
        }

        /**!
         * Sets the team traits matrix
         *
         * \note For AllocationPercentageRemaining tests
         */
        inline void setTeamTraitsMatrix(const Eigen::MatrixXf& team_traits_matrix)
        {
            m_team_traits_matrix = team_traits_matrix;
        }

        /**!
         * Sets the list of tasks
         *
         * \note For MilpScheduler tests
         */
        inline void setTasks(const std::vector<std::shared_ptr<const Task>>& tasks)
        {
            m_tasks = tasks;
        }

        /**!
         * Sets the list of robots
         *
         * \note For MilpScheduler tests
         */
        inline void setRobots(const std::vector<std::shared_ptr<const Robot>>& robots)
        {
            m_robots = robots;
        }

        /**!
         * Sets the schedule parameters
         *
         * \note For MilpScheduler tests
         */
        inline void setScheduleParameters(const std::shared_ptr<const SchedulerParameters>& scheduler_parameters)
        {
            m_scheduler_parameters = scheduler_parameters;
        }

        using GrstapsProblemInputs::createTasks;
        using GrstapsProblemInputs::loadMotionPlanners;
        using GrstapsProblemInputs::loadRobots;
        using GrstapsProblemInputs::loadSpecies;
    };

}  // namespace grstapse::mocks