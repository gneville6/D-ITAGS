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
#include <set>
#include <tuple>
// Local
#include "grstapse/task_allocation/itags/itags_problem_inputs.hpp"

namespace grstapse
{
    //! \brief Container for the inputs to a scheduling problem
    class SchedulerProblemInputs
    {
       public:
        //! Constructor
        SchedulerProblemInputs(const std::shared_ptr<const ItagsProblemInputs>& problem_inputs,
                               const Eigen::MatrixXf& allocation,
                               const std::set<std::pair<unsigned int, unsigned int>>& mutex_constraints);

        /**!
         * Throws an exception if the output from task planning isn't valid
         *
         * Reasons it could be invalid:
         * - mutex constraint uses a index that is out of range of the number of plan tasks
         * - precedence constraint uses a index that is out of range of the number of plan tasks
         */
        void validate() const;

        // Output from Task Allocation
        [[nodiscard]] inline const Eigen::MatrixXf& allocation() const;
        [[nodiscard]] inline const std::set<std::pair<unsigned int, unsigned int>>& mutexConstraints() const;

        // Output from Task Planning
        [[nodiscard]] inline std::vector<std::shared_ptr<const Task>> planTasks() const;
        [[nodiscard]] inline const std::shared_ptr<const Task>& planTask(unsigned int index) const;
        [[nodiscard]] inline unsigned int numberOfPlanTasks() const;
        [[nodiscard]] inline const std::multimap<unsigned int, unsigned int>& precedenceConstraints() const;

        // Module Parameters
        [[nodiscard]] inline const std::shared_ptr<const SchedulerParameters>& schedulerParameters() const;

        // Problem Inputs
        //// Tasks
        //// Robots
        [[nodiscard]] inline const std::vector<std::shared_ptr<const Robot>>& robots() const;
        [[nodiscard]] inline const std::shared_ptr<const Robot>& robot(unsigned int index) const;
        [[nodiscard]] inline unsigned int numberOfRobots() const;
        //// Species
        [[nodiscard]] inline const std::vector<std::shared_ptr<const Species>>& multipleSpecies() const;
        [[nodiscard]] inline const std::shared_ptr<const Species>& individualSpecies(unsigned int index) const;
        [[nodiscard]] inline unsigned int numberOfSpecies() const;
        //// Environments
        [[nodiscard]] inline const std::vector<std::shared_ptr<EnvironmentBase>>& environments() const;
        [[nodiscard]] inline const std::shared_ptr<EnvironmentBase>& environment(unsigned int index) const;
        //// Motion Planners
        [[nodiscard]] inline const std::vector<std::shared_ptr<MotionPlannerBase>>& motionPlanners() const;
        [[nodiscard]] inline const std::shared_ptr<MotionPlannerBase>& motionPlanner(unsigned int index) const;

       private:
        // From task allocation
        Eigen::MatrixXf m_allocation;
        std::set<std::pair<unsigned int, unsigned int>> m_mutex_constraints;
        // todo deadlines?

        std::shared_ptr<const ItagsProblemInputs> m_itags_problem_inputs;
    };

    // Inline functions
    const Eigen::MatrixXf& SchedulerProblemInputs::allocation() const
    {
        return m_allocation;
    }
    const std::set<std::pair<unsigned int, unsigned int>>& SchedulerProblemInputs::mutexConstraints() const
    {
        return m_mutex_constraints;
    }
    std::vector<std::shared_ptr<const Task>> SchedulerProblemInputs::planTasks() const
    {
        return m_itags_problem_inputs->planTasks();
    }
    const std::shared_ptr<const Task>& SchedulerProblemInputs::planTask(unsigned int index) const
    {
        return m_itags_problem_inputs->planTask(index);
    }
    unsigned int SchedulerProblemInputs::numberOfPlanTasks() const
    {
        return m_itags_problem_inputs->numberOfPlanTasks();
    }
    const std::multimap<unsigned int, unsigned int>& SchedulerProblemInputs::precedenceConstraints() const
    {
        return m_itags_problem_inputs->precedenceConstraints();
    }
    const std::shared_ptr<const SchedulerParameters>& SchedulerProblemInputs::schedulerParameters() const
    {
        return m_itags_problem_inputs->schedulerParameters();
    }
    const std::vector<std::shared_ptr<const Robot>>& SchedulerProblemInputs::robots() const
    {
        return m_itags_problem_inputs->robots();
    }
    const std::shared_ptr<const Robot>& SchedulerProblemInputs::robot(unsigned int index) const
    {
        return m_itags_problem_inputs->robot(index);
    }
    unsigned int SchedulerProblemInputs::numberOfRobots() const
    {
        return m_itags_problem_inputs->numberOfRobots();
    }
    const std::vector<std::shared_ptr<const Species>>& SchedulerProblemInputs::multipleSpecies() const
    {
        return m_itags_problem_inputs->multipleSpecies();
    }
    const std::shared_ptr<const Species>& SchedulerProblemInputs::individualSpecies(unsigned int index) const
    {
        return m_itags_problem_inputs->individualSpecies(index);
    }
    unsigned int SchedulerProblemInputs::numberOfSpecies() const
    {
        return m_itags_problem_inputs->numberOfSpecies();
    }
    const std::vector<std::shared_ptr<EnvironmentBase>>& SchedulerProblemInputs::environments() const
    {
        return m_itags_problem_inputs->environments();
    }
    const std::shared_ptr<EnvironmentBase>& SchedulerProblemInputs::environment(unsigned int index) const
    {
        return m_itags_problem_inputs->environment(index);
    }
    const std::vector<std::shared_ptr<MotionPlannerBase>>& SchedulerProblemInputs::motionPlanners() const
    {
        return m_itags_problem_inputs->motionPlanners();
    }
    const std::shared_ptr<MotionPlannerBase>& SchedulerProblemInputs::motionPlanner(unsigned int index) const
    {
        return m_itags_problem_inputs->motionPlanner(index);
    }
}  // namespace grstapse