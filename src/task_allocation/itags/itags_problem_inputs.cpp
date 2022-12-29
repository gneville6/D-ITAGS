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
#include "grstapse/task_allocation/itags/itags_problem_inputs.hpp"

// Local
#include "grstapse/common/search/best_first_search_parameters.hpp"
#include "grstapse/common/utilities/custom_json_conversions.hpp"
#include "grstapse/common/utilities/error.hpp"
#include "grstapse/common/utilities/time_keeper.hpp"
#include "grstapse/geometric_planning/configuration_base.hpp"
#include "grstapse/geometric_planning/ompl/ompl_environment.hpp"
#include "grstapse/geometric_planning/ompl/ompl_motion_planner.hpp"
#include "grstapse/grstaps_problem_inputs.hpp"
#include "grstapse/scheduling/milp/deterministic/deterministic_milp_scheduler.hpp"
#include "grstapse/scheduling/milp/deterministic/deterministic_schedule.hpp"
#include "grstapse/scheduling/milp/milp_scheduler_parameters.hpp"
#include "grstapse/scheduling/scheduler_problem_inputs.hpp"
#include "grstapse/species.hpp"
#include "grstapse/task.hpp"
#include "grstapse/task_allocation/itags/robot_traits_matrix_reduction.hpp"
#include "grstapse/task_allocation/itags/task_allocation_math.hpp"
#include "grstapse/task_planning/sas/sas_action.hpp"

namespace grstapse
{
    ItagsProblemInputs::ItagsProblemInputs(const std::shared_ptr<const GrstapsProblemInputs> &problem_inputs,
                                           const std::vector<unsigned int> &plan_task_indicies,
                                           const std::multimap<unsigned int, unsigned int> &precedence_constraints,
                                           const Eigen::MatrixXf &desired_traits_matrix,
                                           const float schedule_best_makespan,
                                           const float schedule_worst_makespan)
        : m_grstaps_problem_inputs(problem_inputs)
        , m_plan_task_indices(plan_task_indicies)
        , m_precedence_constraints(precedence_constraints)
        , m_desired_traits_matrix(desired_traits_matrix)
        , m_schedule_best_makespan(schedule_best_makespan)
        , m_schedule_worst_makespan(schedule_worst_makespan)
    {}

    std::shared_ptr<ItagsProblemInputs> ItagsProblemInputs::splice(
        const std::shared_ptr<const ItagsProblemInputs> &for_mp_and_species) const
    {
        auto grstaps_inputs =
            m_grstaps_problem_inputs->spliceSpeciesAndMotionPlanners(for_mp_and_species->m_grstaps_problem_inputs);

        return std::make_shared<ItagsProblemInputs>(grstaps_inputs,
                                                    m_plan_task_indices,
                                                    m_precedence_constraints,
                                                    m_desired_traits_matrix,
                                                    m_schedule_best_makespan,
                                                    m_schedule_worst_makespan);
    }

    void ItagsProblemInputs::validate() const
    {
        unsigned int num_plan_task = numberOfPlanTasks();
        for(const std::pair<unsigned int, unsigned int> &constraint: m_precedence_constraints)
        {
            if(constraint.first >= num_plan_task || constraint.second >= num_plan_task)
            {
                throw createLogicError("Precedence constraint out of range of the number of plan tasks");
            }
        }
    }

    std::vector<std::shared_ptr<const Task>> ItagsProblemInputs::planTasks() const
    {
        std::vector<std::shared_ptr<const Task>> rv;
        rv.reserve(m_plan_task_indices.size());
        for(unsigned int task_index: m_plan_task_indices)
        {
            rv.push_back(planTask(task_index));
        }
        return rv;
    }

    std::vector<float> ItagsProblemInputs::planTaskDurations() const
    {
        std::vector<float> rv;
        rv.reserve(m_plan_task_indices.size());
        for(unsigned int task_index: m_plan_task_indices)
        {
            rv.push_back(planTask(task_index)->staticDuration());
        }
        return rv;
    }

    const std::shared_ptr<const Task> &ItagsProblemInputs::planTask(unsigned int index) const
    {
        unsigned int task_index = m_plan_task_indices[index];
        assert(task_index < m_grstaps_problem_inputs->numberOfTasks());
        return m_grstaps_problem_inputs->task(task_index);
    }

    std::vector<std::shared_ptr<const Task>> ItagsProblemInputs::loadTasks(
        const nlohmann::json &j,
        const std::shared_ptr<const GrstapsProblemInputs> &grstaps_problem_inputs)
    {
        if(!j.is_array())
        {
            throw createLogicError("''tasks' must be an array");
        }

        std::vector<std::shared_ptr<const Task>> tasks;
        tasks.reserve(j.size());
        for(const nlohmann::json &task_j: j)
        {
            std::string name = "";
            if(const auto &name_j_itr = task_j.find(constants::k_name); name_j_itr != task_j.end())
            {
                name = *name_j_itr;
            }
            const float duration                           = task_j.at(constants::k_duration);
            const Eigen::VectorXf desired_traits           = task_j.at(constants::k_desired_traits);
            const nlohmann::json &initial_configuration_j  = task_j.at(constants::k_initial_configuration);
            const nlohmann::json &terminal_configuration_j = task_j.at(constants::k_terminal_configuration);

            const std::shared_ptr<const ConfigurationBase> initial_configuration =
                ConfigurationBase::deserializeFromJson(initial_configuration_j);
            grstaps_problem_inputs->checkConfiguration(initial_configuration);

#include <grstapse/common/utilities/custom_json_conversions.hpp>

            const std::shared_ptr<const ConfigurationBase> terminal_configuration =
                ConfigurationBase::deserializeFromJson(terminal_configuration_j);
            grstaps_problem_inputs->checkConfiguration(terminal_configuration);

            tasks.push_back(std::make_shared<const Task>(std::make_shared<SasAction>(name, duration),
                                                         desired_traits,
                                                         initial_configuration,
                                                         terminal_configuration));
        }
        return tasks;
    }

    void from_json(const nlohmann::json &j, ItagsProblemInputs &p)
    {
        // Load/Create Grstaps stuff
        {
            p.m_alpha                   = j.at(constants::k_alpha);
            auto grstaps_problem_inputs = std::make_shared<GrstapsProblemInputs>();

            // Load Environments & Motion Planners
            grstaps_problem_inputs->loadMotionPlanners(j.at(constants::k_motion_planners));

            // Create Mock tasks
            grstaps_problem_inputs->m_tasks = p.loadTasks(j.at(constants::k_tasks), grstaps_problem_inputs);

            // Load Species
            auto [name_to_species_mapping, num_traits] =
                grstaps_problem_inputs->loadSpecies(j.at(constants::k_species));

            // Load Robots
            // Note: cannot use the normal from_json function because the vector of species is needed
            grstaps_problem_inputs->loadRobots(name_to_species_mapping, num_traits, j.at(constants::k_robots));

            // Load Module Parameters
            {
                grstaps_problem_inputs->m_itags_parameters =
                    j.at(constants::k_itags_parameters).get<std::shared_ptr<BestFirstSearchParameters>>();
                if(j.find(constants::k_robot_traits_matrix_reduction) != j.end())
                {
                    grstaps_problem_inputs->m_robot_traits_matrix_reduction =
                        j.at(constants::k_robot_traits_matrix_reduction)
                            .get<std::shared_ptr<RobotTraitsMatrixReduction>>();
                }
                else
                {
                    grstaps_problem_inputs->m_robot_traits_matrix_reduction =
                        std::make_shared<const RobotTraitsMatrixReduction>();
                }
                grstaps_problem_inputs->m_scheduler_parameters =
                    SchedulerParameters::deserializeFromJson(j.at(constants::k_scheduler_parameters));
                // MP parameters are loaded up above
            }
            p.m_grstaps_problem_inputs = grstaps_problem_inputs;
        }
        MilpSchedulerBase::initGurobi(std::dynamic_pointer_cast<const MilpSchedulerParameters>(
            p.m_grstaps_problem_inputs->schedulerParameters()));

        // Load TP Stuff
        j.at(constants::k_plan_task_indices).get_to(p.m_plan_task_indices);

        // TODO(Andrew): custom from/to json for map/multimap with non-string keys (create vector of pairs)
        std::vector<std::pair<unsigned int, unsigned int>> tmp;
        j.at(constants::k_precedence_constraints).get_to(tmp);
        for(const auto &kv: tmp)
        {
            p.m_precedence_constraints.insert(kv);
        }
        p.m_desired_traits_matrix = desiredTraitsMatrix(p.m_grstaps_problem_inputs->m_tasks, p.m_plan_task_indices);

        // Compute makespan for schedule best
        // TODO(Andrew): turn this into a utility/static function somewhere
        {
            // Create empty allocation matrix
            Eigen::MatrixXf allocation(p.numberOfPlanTasks(), p.numberOfRobots());
            allocation.setZero();

            // Pass a shared_ptr that doesn't delete
            auto scheduler_problem_inputs = std::shared_ptr<SchedulerProblemInputs>(
                new SchedulerProblemInputs(std::shared_ptr<ItagsProblemInputs>(&p, [](ItagsProblemInputs *) {}),
                                           allocation,
                                           {}));
            DeterministicMilpScheduler scheduler(scheduler_problem_inputs);
            auto schedule = scheduler.solve();
            if(!schedule)
            {
                throw createLogicError("Schedule best cannot be created. Problem is unsolvable.");
            }
            p.m_schedule_best_makespan = schedule->makespan();
        }

        // Compute makespan for schedule worst
        // TODO(Andrew): turn this into a utility/static function somewhere
        {
            float slowest_speed = std::numeric_limits<float>::infinity();
            for(const std::shared_ptr<const Species> &species: p.multipleSpecies())
            {
                slowest_speed = std::min(slowest_speed, species->speed());
            }
            float longest_path = -1;
            for(const std::shared_ptr<EnvironmentBase> &environment: p.environments())
            {
                longest_path = std::max(longest_path, environment->longestPath());
            }
            const float worst_mp_duration = longest_path / slowest_speed;
            p.m_schedule_worst_makespan   = 0;
            for(const std::shared_ptr<const Task> &task: p.planTasks())
            {
                // Worst mp duration to get to the task and again to execute the task
                p.m_schedule_worst_makespan += 2 * worst_mp_duration + task->staticDuration();
            }
            p.m_schedule_worst_makespan = j.at(constants::k_worst_makespan);
        }

        TimeKeeper::instance().reset(constants::k_scheduling_time);
    }
}  // namespace grstapse