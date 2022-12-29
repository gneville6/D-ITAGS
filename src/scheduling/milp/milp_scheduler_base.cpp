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
#include "grstapse/scheduling/milp/milp_scheduler_base.hpp"

// Local
#include "grstapse/scheduling/milp/milp_scheduler_parameters.hpp"

namespace grstapse {
    unsigned int MilpSchedulerBase::s_num_iterations = 0;
    GRBEnv MilpSchedulerBase::s_environment = GRBEnv(true);
    bool MilpSchedulerBase::s_environment_setup = false;

    MilpSchedulerBase::MilpSchedulerBase(const std::shared_ptr<const SchedulerProblemInputs> &problem_inputs)
            : SchedulerBase(problem_inputs) {}

    void MilpSchedulerBase::initGurobi(const std::shared_ptr<const MilpSchedulerParameters> &parameters) {
        if (!s_environment_setup) {
            // Setup environment
            auto milp_parameters = std::dynamic_pointer_cast<const MilpSchedulerParameters>(parameters);
            s_environment.set(GRB_IntParam_LogToConsole, 0);
            if (milp_parameters->timeout > 0.0f) {
                s_environment.set(GRB_DoubleParam_TimeLimit, 10); //milp_parameters->timeout);
            }
            if (milp_parameters->threads > 0) {
                s_environment.set(GRB_IntParam_Threads, milp_parameters->threads);
            }
            s_environment.set(GRB_DoubleParam_MIPGap, 0.1); //0.4
            s_environment.set(GRB_IntParam_MIPFocus, 3); //1

            s_environment.start();
            s_environment_setup = true;
        }
    }

    unsigned int MilpSchedulerBase::numIterations() {
        return s_num_iterations;
    }

    bool MilpSchedulerBase::createModel(GRBModel &model) {
        if (!createTaskDurations(model)) {
            return false;
        }

        if (!createPrecedenceConstraints(model)) {
            return false;
        }

        if (!createMutexConstraints(model)) {
            return false;
        }

        if (!createInitialTransitions(model)) {
            return false;
        }

        if (!createObjective(model)) {
            return false;
        }

        return true;
    }
}  // namespace grstapse