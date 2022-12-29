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

// Global
#include <fstream>
#include <memory>
// External
#include <nlohmann/json.hpp>
// Local
#include "grstapse/common/search/greedy_best_first_search/greedy_best_first_search.hpp"
#include "grstapse/common/search/hash_memoization.hpp"
#include "grstapse/common/utilities/constants.hpp"
#include "grstapse/common/utilities/matrix_dimensions.hpp"
#include "grstapse/common/utilities/time_keeper.hpp"
#include "grstapse/geometric_planning/motion_planner_base.hpp"
#include "grstapse/geometric_planning/motion_planning_query_result_base.hpp"
#include "grstapse/grstaps_problem_inputs.hpp"
#include "grstapse/robot.hpp"
#include "grstapse/scheduling/milp/deterministic/deterministic_milp_scheduler.hpp"
#include "grstapse/scheduling/milp/deterministic/deterministic_schedule.hpp"
#include "grstapse/scheduling/scheduler_base.hpp"
#include "grstapse/task.hpp"
#include "grstapse/task_allocation/itags/desired_traits_check.hpp"
#include "grstapse/task_allocation/itags/incremental_allocation_generator.hpp"
#include "grstapse/task_allocation/itags/incremental_task_allocation_node.hpp"
#include "grstapse/task_allocation/itags/itags_problem_inputs.hpp"
#include "grstapse/task_allocation/itags/normalized_schedule_quality.hpp"
#include "grstapse/task_allocation/itags/task_allocation_math.hpp"
#include "grstapse/task_allocation/itags/time_extended_task_allocation_quality.hpp"
#include "grstapse/task_allocation/itags/traits_improvement_pruning.hpp"

namespace grstapse
{
    /**!
     * \brief The Incremental Task Allocation Graph Search
     *
     * A heuristic search used for trait-based time extended task allocation problems
     *
     * \tparam HeuristicDeriv The heuristic to be used during search
     * \tparam NodeDeriv the node type that itags will use for the graph search
     *
     * \cite Neville, G., Messing , A., Ravichandar, H., Hutchinson, S.,& Chernova, S. (n.d.). IEEE/RSJ International
     * Conference on Intelligent Robots and Systems (IROS 2021) . In An Interleaved Approach to Trait-Based Task
     * Allocation and Scheduling.
     */
    template <typename HeuristicDeriv = TimeExtendedTaskAllocationQuality<IncrementalTaskAllocationNode>,
              typename NodeDeriv      = IncrementalTaskAllocationNode>
    requires std::derived_from<HeuristicDeriv, HeuristicBase<NodeDeriv>>
    class Itags : public GreedyBestFirstSearch<NodeDeriv>
    {
        using Base = GreedyBestFirstSearch<NodeDeriv>;

       public:
        /**!
         * \brief Constructor
         *
         * @param parameters
         */
        explicit Itags(const std::shared_ptr<const ItagsProblemInputs> &problem_inputs)
            : Base{.parameters = problem_inputs->itagsParameters(),
                   .functors   = {.heuristic = std::make_shared<const HeuristicDeriv>(problem_inputs),
                                  .successor_generator =
                                      std::make_shared<const IncrementalAllocationGenerator<NodeDeriv>>(problem_inputs),
                                  .goal_check  = std::make_shared<const DesiredTraitsCheck<NodeDeriv>>(problem_inputs),
                                  .memoization = std::make_shared<const HashMemoization<NodeDeriv>>(),
                                  .prepruning_method =
                                      std::make_shared<const TraitsImprovementPruning<NodeDeriv>>(problem_inputs),
                                  .postpruning_method = std::make_shared<const NullPruningMethod<NodeDeriv>>()}}
            , m_problem_inputs(problem_inputs)
        {}

        //! \returns Whether the specified problem can be allocated
        [[nodiscard]] bool isAllocatable() const
        {
            // N
            const unsigned int num_robots = m_problem_inputs->numberOfRobots();
            // M
            const unsigned int num_tasks = m_problem_inputs->numberOfPlanTasks();

            // A \in \R^{M \times N}
            Eigen::MatrixXf allocation = Eigen::MatrixXf::Ones(num_tasks, num_robots);

            return traitsMismatchError(*m_problem_inputs->robotTraitsMatrixReduction(),
                                       allocation,
                                       m_problem_inputs->desiredTraitsMatrix(),
                                       m_problem_inputs->teamTraitsMatrix()) == 0;
        }

        std::shared_ptr<NodeDeriv> createRootNode() override
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            const unsigned int num_robots = m_problem_inputs->numberOfRobots();
            const unsigned int num_tasks  = m_problem_inputs->numberOfPlanTasks();
            // Allocation matrix is M X N (number_of_tasks X number_of_robots)
            return std::make_shared<NodeDeriv>(MatrixDimensions{.height = num_tasks, .width = num_robots});
        }

        void writeSolutionToFile(const std::string &filepath, const std::shared_ptr<NodeDeriv> &solution)
        {
            const float motion_planning_time = TimeKeeper::instance().time(constants::k_motion_planning_time);
            const float smp_time             = TimeKeeper::instance().time(constants::k_scheduling_time);
            const float scheduling_time      = smp_time - motion_planning_time;
            const float total_time           = TimeKeeper::instance().time(Base::m_parameters->timer_name);
            const float task_allocation_time = total_time - smp_time;

            nlohmann::json solution_j;
            Eigen::MatrixXf allocation                      = solution->allocation();
            solution_j[constants::k_allocation]             = allocation;
            solution_j[constants::k_makespan]               = solution->schedule()->makespan();
            solution_j[constants::k_precedence_constraints] = m_problem_inputs->precedenceConstraints();

            std::cout << solution->schedule()->makespan() << std::endl;
            std::cout << total_time << std::endl;

            // If a schedule was not already computed then create one
            auto schedule = std::dynamic_pointer_cast<const DeterministicSchedule>(solution->schedule());
            if(!schedule)
            {
                NormalizedScheduleQuality<NodeDeriv> nsq(m_problem_inputs);
                const float makespan = nsq(solution);
                schedule             = std::dynamic_pointer_cast<const DeterministicSchedule>(solution->schedule());
            }
            solution_j[constants::k_precedence_set_mutex_constraints] = schedule->precedenceSetMutexConstraints();

            std::vector<std::vector<unsigned int>> individual_robot_plans(m_problem_inputs->numberOfRobots());

            // Collect and sort task information (name, id, timepoints, coalition, mp)
            const std::vector<std::pair<float, float>> &timepoints = schedule->timepoints();
            {
                nlohmann::json task_list_j;
                for(unsigned int task_nr = 0, num_tasks = m_problem_inputs->numberOfPlanTasks(); task_nr < num_tasks;
                    ++task_nr)
                {
                    const std::shared_ptr<const Task> &task = m_problem_inputs->planTask(task_nr);
                    nlohmann::json task_j;
                    task_j[constants::k_name]             = task->name();
                    task_j[constants::k_id]               = task_nr;
                    task_j[constants::k_start_timepoint]  = timepoints[task_nr].first;
                    task_j[constants::k_finish_timepoint] = timepoints[task_nr].second;
                    std::vector<unsigned int> coalition_ids;
                    std::vector<std::shared_ptr<const Robot>> coalition;
                    for(unsigned int robot_nr = 0, num_robots = m_problem_inputs->numberOfRobots();
                        robot_nr < num_robots;
                        ++robot_nr)
                    {
                        if(allocation(task_nr, robot_nr))
                        {
                            coalition_ids.push_back(robot_nr);
                            coalition.push_back(m_problem_inputs->robot(robot_nr));
                            individual_robot_plans[robot_nr].push_back(task_nr);
                        }
                    }
                    task_j[constants::k_coalition] = coalition_ids;
                    std::shared_ptr<const MotionPlanningQueryResultBase> motion_planning_result =
                        task->motionPlanningQuery(coalition);
                    nlohmann::json mp_j;
                    if(motion_planning_result != NULL)
                    {
                        motion_planning_result->serializeToJson(mp_j);
                    }
                    task_j[constants::k_execution_motion_plan] = mp_j;

                    task_list_j.push_back(task_j);
                }
                solution_j[constants::k_tasks] = task_list_j;
            }

            // Collect and store robot plan information (name, id, individual_plan, transitions)
            {
                nlohmann::json robot_list_j;

                for(unsigned int robot_nr = 1, num_robots = m_problem_inputs->numberOfRobots(); robot_nr < num_robots;
                    ++robot_nr)
                {
                    nlohmann::json robot_j;
                    const std::shared_ptr<const Robot> &robot = m_problem_inputs->robot(robot_nr);
                    robot_j[constants::k_name]                = robot->name();  //!< name
                    robot_j[constants::k_id]                  = robot_nr;       //!< id

                    std::sort(individual_robot_plans[robot_nr].begin(),
                              individual_robot_plans[robot_nr].end(),
                              [&timepoints](const unsigned int &lhs, const unsigned int &rhs)
                              {
                                  return timepoints[lhs].first < timepoints[rhs].first;
                              });
                    robot_j[constants::k_individual_plan] = individual_robot_plans[robot_nr];  //!< individual plan

                    nlohmann::json transition_list_j;
                    if(individual_robot_plans[robot_nr].size() > 0)
                    {
                        nlohmann::json transition_j;
                        // Initial transition
                        std::shared_ptr<const Task> task =
                            m_problem_inputs->planTask(individual_robot_plans[robot_nr][0]);
                        std::shared_ptr<const MotionPlanningQueryResultBase> transition =
                            robot->motionPlanningQuery(task->initialConfiguration());
                        transition->serializeToJson(transition_j);
                        transition_list_j.push_back(transition_j);

                        std::shared_ptr<const Task> &previous_task = task;
                        for(unsigned int i = 1, end = individual_robot_plans[robot_nr].size(); i < end; ++i)
                        {
                            task       = m_problem_inputs->planTask(individual_robot_plans[robot_nr][i]);
                            transition = robot->motionPlanningQuery(previous_task->terminalConfiguration(),
                                                                    task->initialConfiguration());
                            transition_j.clear();
                            transition->serializeToJson(transition_j);
                            transition_list_j.push_back(transition_j);

                            previous_task = task;
                        }
                        robot_j[constants::k_transitions] = transition_list_j;  //!< transitions
                    }
                    robot_list_j.push_back(robot_j);
                }
                solution_j[constants::k_robots] = robot_list_j;
            }

            // Stats
            {
                nlohmann::json stats_j;
                Base::m_statistics->serializeToJson(stats_j);
                stats_j[constants::k_total_time]           = total_time;
                stats_j[constants::k_task_allocation_time] = task_allocation_time;
                stats_j[constants::k_scheduling_time]      = scheduling_time;
                stats_j[constants::k_motion_planning_time] = motion_planning_time;
                solution_j[constants::k_statistics]        = stats_j;

                unsigned int num_motion_plans = 0;
                for(const std::shared_ptr<MotionPlannerBase> &motion_planner: m_problem_inputs->motionPlanners())
                {
                    num_motion_plans += motion_planner->numMotionPlans();
                }
                stats_j[constants::k_num_motion_plans]          = num_motion_plans;
                stats_j[constants::k_num_motion_plan_failures]  = MotionPlannerBase::numFailures();
                stats_j[constants::k_num_scheduling_failures]   = SchedulerBase::numFailures();
                stats_j[constants::k_num_scheduling_iterations] = MilpSchedulerBase::numIterations();
                solution_j[constants::k_statistics]             = stats_j;
            }

            std::ofstream out(filepath);
            out << std::setw(4) << solution_j << std::endl;
            out.close();
        }

       protected:
        std::shared_ptr<const ItagsProblemInputs> m_problem_inputs;
    };
}  // namespace grstapse