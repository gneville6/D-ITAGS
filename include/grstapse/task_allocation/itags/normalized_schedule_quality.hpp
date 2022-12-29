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
#include <set>
#include <tuple>
// Local
#include "grstapse/common/search/heuristic_base.hpp"
#include "grstapse/scheduling/milp/deterministic/deterministic_milp_scheduler.hpp"
#include "grstapse/scheduling/schedule_base.hpp"
#include "grstapse/scheduling/scheduler_problem_inputs.hpp"
#include "grstapse/task_allocation/itags/task_allocation_node_base.hpp"
#include "grstapse/task_allocation/itags/incremental_task_allocation_node.hpp"

namespace grstapse {
    class ItagsProblemInputs;

    class IncrementalTaskAllocationNode;

    /**!
     * Evaluates an allocation based on the quality of the makespan from the associated schedule
     *
     * \see Itags
     *
     * \tparam NodeDeriv The node type that NSQ will be calculated on
     *
     * \cite An  Interleaved  Approach  to  Trait-Based  Task  Allocation and Scheduling}{2021} G. Neville, A. Messing,
     H. Ravichandar, S. Hutchinson, S. Chernova, and Harish. (2021). The International Journal of Robotics Research.}

     * \cite
     */
    template<typename NodeDeriv = IncrementalTaskAllocationNode> requires std::derived_from<NodeDeriv, TaskAllocationNodeBase<NodeDeriv>>
    class NormalizedScheduleQuality : public HeuristicBase<NodeDeriv> {
    public:
        //! \brief Constructor
        NormalizedScheduleQuality(const std::shared_ptr<const ItagsProblemInputs> &problem_inputs)
                : m_problem_inputs(problem_inputs) {}

        //! \returns The quality of the makespan of the associated schedule
        [[nodiscard]] float operator()(const std::shared_ptr<NodeDeriv> &node) const final override {
            if (m_problem_inputs->scheduleWorstMakespan() == 0) {
                node->setNSQ(0);
                return 0;
            }
            node->setNSQ((computeMakespan(node) - m_problem_inputs->scheduleBestMakespan()) /
                         (m_problem_inputs->scheduleWorstMakespan() - m_problem_inputs->scheduleBestMakespan()));
            return node->getNSQ().value();
        }


    protected:
        //! \returns The makespan for the associated schedule of \p node
        [[nodiscard]] virtual float computeMakespan(const std::shared_ptr<NodeDeriv> &node) const {
            // Calculate the Makespan
            auto scheduler_problem_inputs = std::make_shared<SchedulerProblemInputs>(m_problem_inputs,
                                                                                     node->allocation(),
                                                                                     computeMutexConstraints(node));
            DeterministicMilpScheduler scheduler(scheduler_problem_inputs);
            node->m_schedule = scheduler.solve();
            if (!node->m_schedule) {
                return std::numeric_limits<float>::infinity();
            }

            return node->m_schedule->makespan();
        }

        std::set<std::pair<unsigned int, unsigned int>> computeMutexConstraints(
                const std::shared_ptr<NodeDeriv> &node) const {
            // Root node has no mutex constraints
            if (node->parent() == nullptr) {
                return std::set<std::pair<unsigned int, unsigned int>>();
            }

            const Eigen::MatrixXf &allocation = node->allocation();
            return computeMutexConstraints(allocation);
        }

        std::set<std::pair<unsigned int, unsigned int>>
        computeMutexConstraints(const Eigen::MatrixXf &allocation) const {
            if (allocation.isZero()) {
                return std::set<std::pair<unsigned int, unsigned int>>();
            }

            std::set<std::pair<unsigned int, unsigned int>> mutex_constraints;
            const unsigned int num_tasks = allocation.rows();
            const unsigned int num_robots = allocation.cols();
            for (unsigned int robot_nr = 0; robot_nr < num_robots; ++robot_nr) {
                std::vector<unsigned int> allocated_tasks;
                for (unsigned int task_nr = 0; task_nr < num_tasks; ++task_nr) {
                    if (allocation(task_nr, robot_nr)) {
                        allocated_tasks.push_back(task_nr);
                    }
                }
                if (!allocated_tasks.empty()) {
                    for (unsigned int task_i_index = 0, num_assigned_tasks = allocated_tasks.size();
                         task_i_index < num_assigned_tasks;
                         ++task_i_index) {
                        const unsigned int task_i = allocated_tasks[task_i_index];
                        for (unsigned int task_j_index = task_i_index + 1; task_j_index < num_assigned_tasks;
                             ++task_j_index) {
                            const unsigned int task_j = allocated_tasks[task_j_index];
                            mutex_constraints.insert({task_i, task_j});
                        }
                    }
                }
            }
            return mutex_constraints;
        }

        std::shared_ptr<const ItagsProblemInputs> m_problem_inputs;
    };
}  // namespace grstapse