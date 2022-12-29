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
#include "grstapse/scheduling/milp/stochastic/stochastic_milp_scheduler.hpp"

// External
#include <fmt/format.h>
// Local
#include "grstapse/common/utilities/constants.hpp"
#include "grstapse/scheduling/milp/stochastic/stochastic_milp_scheduler_parameters.hpp"
#include "grstapse/scheduling/scheduler_problem_inputs.hpp"

namespace grstapse
{
    StochasticMilpScheduler::StochasticMilpScheduler(
        const std::shared_ptr<const SchedulerProblemInputs>& problem_inputs)
        : MilpSchedulerBase(problem_inputs)
        , m_num_scenario(
              std::dynamic_pointer_cast<const StochasticMilpSchedulerParameters>(problem_inputs->schedulerParameters())
                  ->num_scenarios)
        , m_alpha_q(m_num_scenario * std::dynamic_pointer_cast<const StochasticMilpSchedulerParameters>(
                                         problem_inputs->schedulerParameters())
                                         ->alpha)
    {
        m_subschedulers.reserve(m_num_scenario);
        for(unsigned int i = 0; i < m_num_scenario; ++i)
        {
            m_subschedulers.emplace_back(i, problem_inputs, m_reduced_mutex_constraints);
        }
    }

    bool StochasticMilpScheduler::createTaskDurations(GRBModel& model)
    {
        for(ScenarioMilpSubscheduler& subscheduler: m_subschedulers)
        {
            if(!subscheduler.createTaskDurations(model))
            {
                return false;
            }
        }
        return true;
    }

    bool StochasticMilpScheduler::createPrecedenceConstraints(GRBModel& model)
    {
        for(ScenarioMilpSubscheduler& subscheduler: m_subschedulers)
        {
            if(!subscheduler.createPrecedenceConstraints(model))
            {
                return false;
            }
        }
        return true;
    }

    bool StochasticMilpScheduler::createMutexConstraints(GRBModel& model)
    {
        for(ScenarioMilpSubscheduler& subscheduler: m_subschedulers)
        {
            if(!subscheduler.createMutexConstraints(model))
            {
                return false;
            }
        }
        return true;
    }

    bool StochasticMilpScheduler::createInitialTransitions(GRBModel& model)
    {
        for(ScenarioMilpSubscheduler& subscheduler: m_subschedulers)
        {
            if(!subscheduler.createInitialTransitions(model))
            {
                return false;
            }
        }
        return true;
    }

    bool StochasticMilpScheduler::createObjective(GRBModel& model)
    {
        // Set all optimization to minimize (is the default, but we explicitly set anyway)
        model.set(GRB_IntAttr_ModelSense, GRB_MINIMIZE);

        // Top level of the objective is minimizing makespan
        GRBVar makespan = model.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS, constants::k_makespan);
        for(ScenarioMilpSubscheduler& subscheduler: m_subschedulers)
        {
            if(!subscheduler.createObjective(model))
            {
                return false;
            }
        }

        GRBLinExpr alpha_summation;
        for(unsigned int i = 0; i < m_num_scenario; ++i)
        {
            alpha_summation += model.getVarByName(fmt::format("y_{0:d}", i));
        }
        model.addConstr(m_alpha_q <= alpha_summation);
        model.setObjective(GRBLinExpr(makespan));
        return true;
    }

    std::shared_ptr<const ScheduleBase> StochasticMilpScheduler::computeSchedule()
    {
        // TODO(andrew)
        throw std::logic_error("Not implemented");
    }
}  // namespace grstapse