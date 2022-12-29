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
#include <vector>
// External
#include <Eigen/Core>
// Local
#include "grstapse/grstaps_problem_inputs.hpp"

namespace grstapse
{
    // Forward Declarations
    class Task;

    /**!
     * Contain for the inputs to an ITAGS problem
     */
    class ItagsProblemInputs
    {
       public:
        //! For json
        ItagsProblemInputs() = default;

        //! Constructor
        ItagsProblemInputs(const std::shared_ptr<const GrstapsProblemInputs> &problem_inputs,
                           const std::vector<unsigned int> &plan_task_indicies,
                           const std::multimap<unsigned int, unsigned int> &precedence_constraints,
                           const Eigen::MatrixXf &desired_traits_matrix,
                           const float schedule_best_makespan,
                           const float schedule_worst_makespan);

        std::shared_ptr<ItagsProblemInputs> splice(
            const std::shared_ptr<const ItagsProblemInputs> &for_mp_and_species) const;

        /**!
         * Throws an exception if the output from task planning isn't valid
         *
         * Reasons it could be invalid:
         * - precedence constraint uses a index that is out of range of the number of plan tasks
         */
        void validate() const;

        // Output from Task Planning
        [[nodiscard]] std::vector<std::shared_ptr<const Task>> planTasks() const;

        [[nodiscard]] std::vector<float> planTaskDurations() const;

        [[nodiscard]] const std::shared_ptr<const Task> &planTask(unsigned int index) const;

        [[nodiscard]] inline unsigned int numberOfPlanTasks() const;

        [[nodiscard]] inline const std::multimap<unsigned int, unsigned int> &precedenceConstraints() const;

        [[nodiscard]] inline const Eigen::MatrixXf &desiredTraitsMatrix() const;

        [[nodiscard]] inline float scheduleBestMakespan() const;

        [[nodiscard]] inline float scheduleWorstMakespan() const;

        // Module Parameters
        [[nodiscard]] inline const std::shared_ptr<const BestFirstSearchParameters> &itagsParameters() const;

        [[nodiscard]] inline const std::shared_ptr<const RobotTraitsMatrixReduction> &robotTraitsMatrixReduction()
            const;

        [[nodiscard]] inline const std::shared_ptr<const SchedulerParameters> &schedulerParameters() const;

        // Problem Inputs
        //// Tasks
        //// Robots
        [[nodiscard]] inline const std::vector<std::shared_ptr<const Robot>> &robots() const;

        [[nodiscard]] inline const std::shared_ptr<const Robot> &robot(unsigned int index) const;

        [[nodiscard]] inline unsigned int numberOfRobots() const;

        //// Species
        [[nodiscard]] inline const std::vector<std::shared_ptr<const Species>> &multipleSpecies() const;

        [[nodiscard]] inline const std::shared_ptr<const Species> &individualSpecies(unsigned int index) const;

        [[nodiscard]] inline unsigned int numberOfSpecies() const;

        //// Team Traits Matrix
        [[nodiscard]] inline const Eigen::MatrixXf &teamTraitsMatrix() const;

        [[nodiscard]] inline unsigned int numberOfTraits() const;

        //// Environments
        [[nodiscard]] inline const std::vector<std::shared_ptr<EnvironmentBase>> &environments() const;

        [[nodiscard]] inline const std::shared_ptr<EnvironmentBase> &environment(unsigned int index) const;

        //// Motion Planners
        [[nodiscard]] inline const std::vector<std::shared_ptr<MotionPlannerBase>> &motionPlanners() const;

        [[nodiscard]] inline const std::shared_ptr<MotionPlannerBase> &motionPlanner(unsigned int index) const;

        float m_alpha;
        float m_schedule_best_makespan;
        float m_schedule_worst_makespan;

       protected:
        std::vector<std::shared_ptr<const Task>> loadTasks(
            const nlohmann::json &j,
            const std::shared_ptr<const GrstapsProblemInputs> &grstaps_problem_inputs);

        // From task planning
        std::vector<unsigned int> m_plan_task_indices;
        std::multimap<unsigned int, unsigned int> m_precedence_constraints;
        Eigen::MatrixXf m_desired_traits_matrix;



        std::shared_ptr<const GrstapsProblemInputs> m_grstaps_problem_inputs;

        friend void from_json(const nlohmann::json &j, ItagsProblemInputs &p);
    };

    void from_json(const nlohmann::json &j, ItagsProblemInputs &p);

    // Inline functions
    unsigned int ItagsProblemInputs::numberOfPlanTasks() const
    {
        return m_plan_task_indices.size();
    }

    const std::multimap<unsigned int, unsigned int> &ItagsProblemInputs::precedenceConstraints() const
    {
        return m_precedence_constraints;
    }

    const Eigen::MatrixXf &ItagsProblemInputs::desiredTraitsMatrix() const
    {
        return m_desired_traits_matrix;
    }

    float ItagsProblemInputs::scheduleBestMakespan() const
    {
        return m_schedule_best_makespan;
    }

    float ItagsProblemInputs::scheduleWorstMakespan() const
    {
        return m_schedule_worst_makespan;
    }

    const std::shared_ptr<const BestFirstSearchParameters> &ItagsProblemInputs::itagsParameters() const
    {
        return m_grstaps_problem_inputs->itagsParameters();
    }

    const std::shared_ptr<const RobotTraitsMatrixReduction> &ItagsProblemInputs::robotTraitsMatrixReduction() const
    {
        return m_grstaps_problem_inputs->robotTraitsMatrixReduction();
    }

    const std::shared_ptr<const SchedulerParameters> &ItagsProblemInputs::schedulerParameters() const
    {
        return m_grstaps_problem_inputs->schedulerParameters();
    }

    const std::vector<std::shared_ptr<const Robot>> &ItagsProblemInputs::robots() const
    {
        return m_grstaps_problem_inputs->robots();
    }

    const std::shared_ptr<const Robot> &ItagsProblemInputs::robot(unsigned int index) const
    {
        return m_grstaps_problem_inputs->robot(index);
    }

    unsigned int ItagsProblemInputs::numberOfRobots() const
    {
        return m_grstaps_problem_inputs->numberOfRobots();
    }

    const std::vector<std::shared_ptr<const Species>> &ItagsProblemInputs::multipleSpecies() const
    {
        return m_grstaps_problem_inputs->multipleSpecies();
    }

    const std::shared_ptr<const Species> &ItagsProblemInputs::individualSpecies(unsigned int index) const
    {
        return m_grstaps_problem_inputs->individualSpecies(index);
    }

    unsigned int ItagsProblemInputs::numberOfSpecies() const
    {
        return m_grstaps_problem_inputs->numberOfSpecies();
    }

    const Eigen::MatrixXf &ItagsProblemInputs::teamTraitsMatrix() const
    {
        return m_grstaps_problem_inputs->teamTraitsMatrix();
    }

    unsigned int ItagsProblemInputs::numberOfTraits() const
    {
        return m_grstaps_problem_inputs->numberOfTraits();
    }

    const std::vector<std::shared_ptr<EnvironmentBase>> &ItagsProblemInputs::environments() const
    {
        return m_grstaps_problem_inputs->environments();
    }

    const std::shared_ptr<EnvironmentBase> &ItagsProblemInputs::environment(unsigned int index) const
    {
        return m_grstaps_problem_inputs->environment(index);
    }

    const std::vector<std::shared_ptr<MotionPlannerBase>> &ItagsProblemInputs::motionPlanners() const
    {
        return m_grstaps_problem_inputs->motionPlanners();
    }

    const std::shared_ptr<MotionPlannerBase> &ItagsProblemInputs::motionPlanner(unsigned int index) const
    {
        return m_grstaps_problem_inputs->motionPlanner(index);
    }
}  // namespace grstapse