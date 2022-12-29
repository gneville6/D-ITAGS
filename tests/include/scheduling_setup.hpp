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

// Project
#include <grstapse/common/utilities/constants.hpp>
#include <grstapse/geometric_planning/ompl/ompl_environment.hpp>
#include <grstapse/geometric_planning/ompl/ompl_motion_planner.hpp>
#include <grstapse/geometric_planning/ompl/ompl_motion_planner_parameters.hpp>
#include <grstapse/geometric_planning/ompl/se2_state_ompl_configuration.hpp>
#include <grstapse/robot.hpp>
#include <grstapse/scheduling/milp/deterministic/deterministic_milp_scheduler.hpp>
#include <grstapse/scheduling/milp/deterministic/deterministic_schedule.hpp>
#include <grstapse/scheduling/milp/milp_scheduler_parameters.hpp>
#include <grstapse/scheduling/scheduler_problem_inputs.hpp>
#include <grstapse/task.hpp>
#include <grstapse/task_allocation/itags/itags_problem_inputs.hpp>
#include <grstapse/task_planning/sas/sas_action.hpp>
// Local
#include "mock_deterministic_milp_scheduler.hpp"
#include "mock_grstaps_problem_inputs.hpp"

namespace grstapse::unittests
{
    // region Utility enums and functions
    // region enums
    enum class PlanOption : uint8_t
    {
        e_total_order,
        e_branch,
        e_diamond,
        e_parallel,
        e_complex
    };

    enum class AllocationOption : uint8_t
    {
        e_none,
        e_identity,
        e_multi_task_robot,
        e_multi_robot_task,
        e_complex,
        e_complex2
    };

    // (Turtlebots)
    enum class SpeciesOption : uint8_t
    {
        e_burger,
        e_waffle
    };
    // endregion

    // region plan options
    void createTotalOrderPlanInputs(std::vector<std::shared_ptr<const Task>>& tasks,
                                    std::multimap<unsigned int, unsigned int>& precedence_constraints);

    void createBranchPlanInputs(std::vector<std::shared_ptr<const Task>>& tasks,
                                std::multimap<unsigned int, unsigned int>& precedence_constraints);

    void createDiamondPlanInputs(std::vector<std::shared_ptr<const Task>>& tasks,
                                 std::multimap<unsigned int, unsigned int>& precedence_constraints);

    void createParallelPlanInputs(std::vector<std::shared_ptr<const Task>>& tasks,
                                  std::multimap<unsigned int, unsigned int>& precedence_constraints);

    void createComplexPlanInputs(std::vector<std::shared_ptr<const Task>>& tasks,
                                 std::multimap<unsigned int, unsigned int>& precedence_constraints);
    // endregion

    // region robots
    void createHomogeneousRobots(SpeciesOption species_option,
                                 unsigned int num,
                                 std::vector<std::shared_ptr<const Robot>>& robots);

    void createHeterogeneousRobots(const std::vector<SpeciesOption>& species_options,
                                   std::vector<std::shared_ptr<const Robot>>& robots);
    // endregion

    /**!
     * Utility function to create the scheduler problem inputs
     */
    std::shared_ptr<SchedulerProblemInputs> createSchedulerProblemInputs(PlanOption plan_option,
                                                                         AllocationOption allocation_option,
                                                                         bool homogeneous);
    // endregion

}  // namespace grstapse::unittests