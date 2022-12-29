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
#include "grstapse/scheduling/milp/deterministic/deterministic_milp_scheduler.hpp"

// External
#include <fmt/format.h>
// Local
#include "grstapse/common/utilities/constants.hpp"
#include "grstapse/common/utilities/error.hpp"
#include "grstapse/geometric_planning/configuration_base.hpp"
#include "grstapse/geometric_planning/ompl/ompl_environment.hpp"
#include "grstapse/geometric_planning/ompl/ompl_motion_planner.hpp"
#include "grstapse/robot.hpp"
#include "grstapse/scheduling/milp/deterministic/deterministic_milp_scheduler_parameters.hpp"
#include "grstapse/scheduling/milp/deterministic/deterministic_schedule.hpp"
#include "grstapse/scheduling/scheduler_problem_inputs.hpp"
#include "grstapse/species.hpp"
#include "grstapse/task.hpp"

namespace grstapse {
    DeterministicMilpScheduler::DeterministicMilpScheduler(
            const std::shared_ptr<const SchedulerProblemInputs> &problem_inputs)
            : DeterministicMilpSchedulerBase(problem_inputs, m_placeholder_reduced_mutex_constraints) {
        if (!s_environment_setup) {
            initGurobi(
                    std::dynamic_pointer_cast<const MilpSchedulerParameters>(m_problem_inputs->schedulerParameters()));
        }
    }

    void DeterministicMilpScheduler::recomputEnviroment() {
        s_environment_setup = false;
    }

    std::shared_ptr<const ScheduleBase> DeterministicMilpScheduler::computeSchedule() {
        auto milp_parameters =
                std::dynamic_pointer_cast<const MilpSchedulerParameters>(m_problem_inputs->schedulerParameters());
        if (milp_parameters->compute_transition_duration_heuristic) {
            if (!computeInitialTransitionHeuristicDurations() || !computeTransitionHeuristicDurations()) {
                return nullptr;
            }
        }

        while (true) {
            ++s_num_iterations;
            GRBModel model(s_environment);
            if (!createModel(model)) {
                ++s_num_failures;
                return nullptr;
            }

            // Optimize model
            model.optimize();

            // Check status
            if (model.get(GRB_IntAttr_Status) != GRB_OPTIMAL) {
                ++s_num_failures;
                return nullptr;
            }

            if (checkAndUpdateTransitions()) {
                return createSchedule(model);
            }
        }
    }

    std::string DeterministicMilpScheduler::createTaskStartName(unsigned int task_nr) const {
        return fmt::format("ts_{0:d}", task_nr);
    }

    std::string DeterministicMilpScheduler::createTaskFinishName(unsigned int task_nr) const {
        return fmt::format("tf_{0:d}", task_nr);
    }

    std::string DeterministicMilpScheduler::createDurationConstraintName(unsigned int task_nr) const {
        return fmt::format("dc_{0:d}", task_nr);
    }

    std::string DeterministicMilpScheduler::createPrecedenceConstraintName(unsigned int i, unsigned int j) const {
        return fmt::format("pc_({0:d},{1:d})", i, j);
    }

    void DeterministicMilpScheduler::addMutexConstraint(GRBModel &model,
                                                        const unsigned int i,
                                                        const double i_to_j_transition_duration,
                                                        const unsigned int j,
                                                        const double j_to_i_transition_duration) {
        const std::string p_ij_name = fmt::format("p_({0:d},{1:d})", i, j);
        GRBVar p_ij = model.addVar(0.0, 1.0, 0.0, GRB_BINARY, p_ij_name);
        if (m_reduced_mutex_constraints.contains(p_ij_name)) {
            m_reduced_mutex_constraints[p_ij_name].variable = p_ij;
        } else {
            m_reduced_mutex_constraints[p_ij_name] =
                    MutexConstraintInfo{.task_i = i, .task_j = j, .variable_name = p_ij_name, .variable = p_ij};
        }

        // i -> j
        model.addGenConstrIndicator(
                p_ij,
                1,
                m_tasks_timepoints[j].start >= m_tasks_timepoints[i].finish + i_to_j_transition_duration,
                fmt::format("tc_({0:d},{1:d})", i, j));

        // j -> i
        model.addGenConstrIndicator(
                p_ij,
                0,
                m_tasks_timepoints[i].start >= m_tasks_timepoints[j].finish + j_to_i_transition_duration,
                fmt::format("tc_({0:d},{1:d})", j, i));
    }

    float DeterministicMilpScheduler::computeTaskDuration(unsigned int task_nr,
                                                          const std::vector<std::shared_ptr<const Robot>> &coalition) {
        return m_problem_inputs->planTask(task_nr)->computeDuration(coalition);
    }

    std::string DeterministicMilpScheduler::createInitialTransitionConstraintName(unsigned int task_nr) const {
        return fmt::format("itc_{0:d}", task_nr);
    }

    bool DeterministicMilpScheduler::isInitialTransitionMemoized(
            const std::shared_ptr<const ConfigurationBase> &configuration,
            const std::shared_ptr<const Robot> &robot) const {
        return robot->isMemoized(configuration);
    }

    float DeterministicMilpScheduler::computeInitialTransitionDuration(
            const std::shared_ptr<const ConfigurationBase> &configuration,
            const std::shared_ptr<const Robot> &robot) const {
        return robot->durationQuery(configuration);
    }

    bool DeterministicMilpScheduler::isTransitionMemoized(
            const std::shared_ptr<const ConfigurationBase> &initial_configuration,
            const std::shared_ptr<const ConfigurationBase> &goal_configuration,
            const std::shared_ptr<const Robot> &robot) const {
        return robot->isMemoized(initial_configuration, goal_configuration);
    }

    float DeterministicMilpScheduler::computeTransitionDuration(
            const std::shared_ptr<const ConfigurationBase> &initial_configuration,
            const std::shared_ptr<const ConfigurationBase> &goal_configuration,
            const std::shared_ptr<const Robot> &robot) const {
        return robot->durationQuery(initial_configuration, goal_configuration);
    }

    bool DeterministicMilpScheduler::createObjective(GRBModel &model) {
        // Set all optimization to minimize (is the default, but we explicitly set anyway)
        model.set(GRB_IntAttr_ModelSense, GRB_MINIMIZE);

        // Top level of the objective is minimizing makespan
        m_makespan = model.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS, constants::k_makespan);
        model.addGenConstrMax(m_makespan, m_task_finishes.get(), m_tasks_timepoints.size());

        if (std::dynamic_pointer_cast<const DeterministicMilpSchedulerParameters>(
                m_problem_inputs->schedulerParameters())
                ->use_hierarchical_objective) {
            // Note: lower priority objectives cannot degrade higher priority objectives
            model.setObjectiveN(GRBLinExpr(m_makespan), 0, 1);  //!< objective, index, priority

            // Bottom level of the object is starting each task as soon as possible (so minimizing start time)
            for (unsigned int i = 0; i < m_tasks_timepoints.size(); ++i) {
                model.setObjectiveN(GRBLinExpr(m_tasks_timepoints[i].start), i + 1, 0);  //!< objective, index, priority
            }
        } else {
            model.setObjective(GRBLinExpr(m_makespan));
        }
        return true;
    }

    bool DeterministicMilpScheduler::checkAndUpdateTransitions() {
        bool no_heuristic = true;
        const unsigned int num_robots = m_problem_inputs->numberOfRobots();
        const Eigen::MatrixXf &allocation = m_problem_inputs->allocation();

        // Sort by order of start
        std::sort(m_tasks_timepoints.begin(),
                  m_tasks_timepoints.end(),
                  [](const TaskVariableInfo &lhs, const TaskVariableInfo &rhs) {
                      return lhs.start.get(GRB_DoubleAttr_X) < rhs.start.get(GRB_DoubleAttr_X);
                  });

        std::vector<int> previous_task(num_robots, -1);
        std::vector<std::shared_ptr<const ConfigurationBase>> previous_configurations;
        previous_configurations.reserve(num_robots);
        for (const std::shared_ptr<const Robot> &robot: m_problem_inputs->robots()) {
            previous_configurations.push_back(robot->initialConfiguration());
        }

        for (const TaskVariableInfo &task_timepoint: m_tasks_timepoints) {
            const std::shared_ptr<const Task> &task = m_problem_inputs->planTask(task_timepoint.task_nr);
            const std::shared_ptr<const ConfigurationBase> &initial_configuration = task->initialConfiguration();
            const std::shared_ptr<const ConfigurationBase> &terminal_configuration = task->terminalConfiguration();

            for (unsigned int robot_nr: task_timepoint.coalition) {
                TaskTransitionInfo &task_transition_info =
                        previous_task[robot_nr] < 0 ? m_initial_transition_info[task_timepoint.task_nr]
                                                    : m_transition_info[previous_task[robot_nr]][task_timepoint.task_nr];
                for (RobotTaskTransitionInfo &robot_task_transition_info: task_transition_info) {
                    if (robot_task_transition_info.robot_nr != robot_nr) {
                        continue;
                    }
                    switch (robot_task_transition_info.computation_status) {
                        case TransitionComputationStatus::e_none:
                        case TransitionComputationStatus::e_heuristic: {
                            const float transition_duration =
                                    m_problem_inputs->robot(robot_nr)->durationQuery(previous_configurations[robot_nr],
                                                                                     initial_configuration);
                            robot_task_transition_info.computation_status = TransitionComputationStatus::e_success;
                            robot_task_transition_info.duration = transition_duration;
                            no_heuristic = false;
                            break;
                        }
                        case TransitionComputationStatus::e_failed: {
                            throw createLogicError(
                                    fmt::format("Request for failed transition (task nr: {0:d}; robot nr: {1:d})",
                                                task_timepoint.task_nr,
                                                robot_nr));
                        }
                        default: {
                            // previously computed
                            break;
                        }
                    }
                    // Set terminal configuration (initial -> terminal is handled by task duration)
                    previous_configurations[robot_nr] = terminal_configuration;
                    previous_task[robot_nr] = task_timepoint.task_nr;
                    break;
                }
            }
        }

        return no_heuristic;
    }

    std::shared_ptr<const DeterministicSchedule> DeterministicMilpScheduler::createSchedule(GRBModel &model) {
        const double makespan = m_makespan.get(GRB_DoubleAttr_X);

        std::vector<std::pair<float, float>> timepoints(m_tasks_timepoints.size());
        for (const TaskVariableInfo &task_timepoints: m_tasks_timepoints) {
            timepoints[task_timepoints.task_nr] =
                    std::pair(task_timepoints.start.get(GRB_DoubleAttr_X),
                              task_timepoints.finish.get(GRB_DoubleAttr_X));
        }

        std::vector<std::pair<unsigned int, unsigned int>> precedence_set_mutex_constraints;
        precedence_set_mutex_constraints.reserve(m_reduced_mutex_constraints.size());
        for (auto&[key, info]: m_reduced_mutex_constraints) {
            if (model.getVarByName(info.variable_name).get(GRB_DoubleAttr_X) > 0.5f) {
                precedence_set_mutex_constraints.push_back(std::pair(info.task_i, info.task_j));
            } else {
                precedence_set_mutex_constraints.push_back(std::pair(info.task_j, info.task_i));
            }
        }

        return std::make_shared<const DeterministicSchedule>(makespan, timepoints, precedence_set_mutex_constraints);
    }
}  // namespace grstapse