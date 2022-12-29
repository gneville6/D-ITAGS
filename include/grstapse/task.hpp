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
// External
#include <Eigen/Core>

namespace grstapse
{
    // Forward Declarations
    class OmplMotionPlanner;
    class Robot;
    class SasAction;
    class ConfigurationBase;
    class MotionPlanningQueryResultBase;

    /**!
     * Container for all of the information in a task (both symbolic and geometric)
     *
     * \note FCPOP (our task planning layer) uses the term Action as that is prevalent in the task planning literature
     *       and so the symbolic part of a task may be referred to as an Action through the code
     */
    class Task
    {
       public:
        /**!
         * Constructor
         *
         * \param symbolic_action The symbolic component of the task
         * \param desired_traits The desired traits for the task
         * \param initial_configuration The configuration space that a robot/coalition needs to be in to start this task
         * \param terminal_configuration The configuration space that a robot/coalition needs to be in to complete this
         * task
         */
        Task(const std::shared_ptr<SasAction>& symbolic_action,
             const Eigen::VectorXf& desired_traits,
             const std::shared_ptr<const ConfigurationBase>& initial_configuration,
             const std::shared_ptr<const ConfigurationBase>& terminal_configuration);

        //! \returns The symbolic action/task
        [[nodiscard]] inline const std::shared_ptr<SasAction>& symbolicAction() const;

        //! \returns The name of the task
        [[nodiscard]] const std::string& name() const;

        //! \returns The static duration of the task
        [[nodiscard]] float staticDuration() const;

        //! \returns The desired traits for the task
        [[nodiscard]] inline const Eigen::VectorXf& desiredTraits() const;

        /**!
         * \returns A configuration space that a robot or coalition needs to be in to start the task (can be a state,
         *          set of states, or state space)
         */
        [[nodiscard]] inline const std::shared_ptr<const ConfigurationBase>& initialConfiguration() const;

        /**!
         * \returns A configuration space that a robot or coalition needs to be in to complete the task (can be a
         *          state, set of states, or state space)
         */
        [[nodiscard]] inline const std::shared_ptr<const ConfigurationBase>& terminalConfiguration() const;

        //! Returns the motion plan for executing this task
        [[nodiscard]] std::shared_ptr<const MotionPlanningQueryResultBase> motionPlanningQuery(
            const std::vector<std::shared_ptr<const Robot>>& coalition) const;

        //! Computes the duration of a task based on a specified coalition
        [[nodiscard]] float computeDuration(const std::vector<std::shared_ptr<const Robot>>& coalition) const;

       private:
        std::shared_ptr<SasAction> m_symbolic_action;
        Eigen::VectorXf m_desired_traits;
        std::shared_ptr<const ConfigurationBase> m_initial_configuration;
        std::shared_ptr<const ConfigurationBase> m_terminal_configuration;
    };

    // Inline Functions
    const std::shared_ptr<SasAction>& Task::symbolicAction() const
    {
        return m_symbolic_action;
    }

    const Eigen::VectorXf& Task::desiredTraits() const
    {
        return m_desired_traits;
    }

    const std::shared_ptr<const ConfigurationBase>& Task::initialConfiguration() const
    {
        return m_initial_configuration;
    }

    const std::shared_ptr<const ConfigurationBase>& Task::terminalConfiguration() const
    {
        return m_terminal_configuration;
    }
}  // namespace grstapse