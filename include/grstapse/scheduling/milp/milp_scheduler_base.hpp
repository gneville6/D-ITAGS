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
#include <gurobi_c++.h>
// Local
#include "grstapse/scheduling/scheduler_base.hpp"

namespace grstapse
{
    // Forward Declaration
    class MilpSchedulerParameters;

    /**!
     * Abstract base class for scheduling algorithms that use MILP formulations
     */
    class MilpSchedulerBase : public SchedulerBase
    {
       public:
        //! Constructor
        explicit MilpSchedulerBase(const std::shared_ptr<const SchedulerProblemInputs>& problem_inputs);

        /**!
         * Sets up the gurobi environment
         *
         * \todo(Andrew): check if licensing failed and then return bool?
         */
        static void initGurobi(const std::shared_ptr<const MilpSchedulerParameters>& parameters);

        //! \returns The number of times a MILP optimization was run
        [[nodiscard]] static unsigned int numIterations();

       protected:
        /**!
         * Creates a MILP model for gurobi to solve
         *
         * \param model Reference to the model that is being constructed
         * \returns Whether the model was successfully created
         */
        virtual bool createModel(GRBModel& model);

        /**!
         * Adds temporal constraints to \p model for the durations of the tasks to schedule
         *
         * \param model Reference to the model that is being constructed
         * \returns Whether the constraints were successfully added
         */
        virtual bool createTaskDurations(GRBModel& model) = 0;

        /**!
         * Adds precedence constraints to \p model
         *
         * \param model Reference to the model that is being constructed
         * \returns Whether the constraints were successfully added
         */
        virtual bool createPrecedenceConstraints(GRBModel& model) = 0;

        /**!
         * Adds mutex constraints to \p model
         *
         * \param model Reference to the model that is being constructed
         * \returns Whether the constraints were successfully added
         */
        virtual bool createMutexConstraints(GRBModel& model) = 0;

        /**!
         * Adds temporal constraints to \p model for the transition of each robot moving from its initial configuration
         * to its first assigned task
         *
         * \param model Reference to the model that is being constructed
         * \returns Whether the constraints were successfully added
         */
        virtual bool createInitialTransitions(GRBModel& model) = 0;

        /**!
         * Adds the optimization function and constraints for the optimization function
         *
         * \param model Reference to the model that is being constructed
         * \returns Whether the constraints were successfully added
         */
        virtual bool createObjective(GRBModel& model) = 0;

        static unsigned int s_num_iterations;
        static GRBEnv s_environment;
        static bool s_environment_setup;
    };

}  // namespace grstapse