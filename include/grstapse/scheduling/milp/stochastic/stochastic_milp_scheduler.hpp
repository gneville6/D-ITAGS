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
#include "grstapse/scheduling/milp/stochastic/scenario_milp_subscheduler.hpp"

namespace grstapse
{
    /**!
     * \brief Uses a MILP formulation to solve a stochastic robot scheduling problem
     *
     * \note We use gurobi as our MILP solver
     */
    class StochasticMilpScheduler : public MilpSchedulerBase
    {
       public:
        //! Constructor
        explicit StochasticMilpScheduler(const std::shared_ptr<const SchedulerProblemInputs>& problem_inputs);

       protected:
        //! \copydoc MilpSchedulerBase
        bool createTaskDurations(GRBModel& model) override;

        //! \copydoc MilpSchedulerBase
        bool createPrecedenceConstraints(GRBModel& model) override;

        //! \copydoc MilpSchedulerBase
        bool createMutexConstraints(GRBModel& model) override;

        //! \copydoc MilpSchedulerBase
        bool createInitialTransitions(GRBModel& model) override;

        //! \copydoc MilpSchedulerBase
        bool createObjective(GRBModel& model) override;

        //! \copydoc MilpSchedulerBase
        std::shared_ptr<const ScheduleBase> computeSchedule() override;

       protected:
        std::vector<ScenarioMilpSubscheduler> m_subschedulers;
        robin_hood::unordered_map<std::string, MutexConstraintInfo> m_reduced_mutex_constraints;
        unsigned int m_num_scenario;
        float m_alpha_q;  //!< alpha * q
    };

}  // namespace grstapse