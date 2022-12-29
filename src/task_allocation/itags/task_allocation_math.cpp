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
#include "grstapse/task_allocation/itags/task_allocation_math.hpp"
#include <iostream>
// Local
#include "grstapse/task.hpp"
#include "grstapse/task_allocation/itags/robot_traits_matrix_reduction.hpp"

namespace grstapse {
    Eigen::MatrixXf desiredTraitsMatrix(const std::vector<std::shared_ptr<const Task>> &tasks,
                                        const std::vector<unsigned int> &plan_task_indicies) {
        if (tasks.empty() || plan_task_indicies.empty()) {
            return Eigen::MatrixXf();
        }

        const unsigned int num_traits = tasks.front()->desiredTraits().size();

        Eigen::MatrixXf rv(plan_task_indicies.size(), num_traits);
        unsigned int row_nr = 0;
        for (const unsigned int index: plan_task_indicies) {
            rv.row(row_nr++) = tasks[index]->desiredTraits();
        }
        return rv;
    }

    Eigen::MatrixXf allocatedTraitsMatrix(const RobotTraitsMatrixReduction &robot_traits_matrix_reduction,
                                          const Eigen::MatrixXf &allocation,
                                          const Eigen::MatrixXf &robot_traits_matrix) {
        return robot_traits_matrix_reduction.reduce(allocation, robot_traits_matrix);
    }

    Eigen::MatrixXf traitsMismatchMatrix(const RobotTraitsMatrixReduction &robot_traits_matrix_reduction,
                                         const Eigen::MatrixXf &allocation,
                                         const Eigen::MatrixXf &desired_traits_matrix,
                                         const Eigen::MatrixXf &robot_traits_matrix) {
        // A * Q
        Eigen::MatrixXf allocated_traits_matrix =
                allocatedTraitsMatrix(robot_traits_matrix_reduction, allocation, robot_traits_matrix);

        // E(A) = Y - A * Q
        return desired_traits_matrix - allocated_traits_matrix;
    }

    Eigen::MatrixXf positiveOnlyTraitsMismatchMatrix(const RobotTraitsMatrixReduction &robot_traits_matrix_reduction,
                                                     const Eigen::MatrixXf &allocation,
                                                     const Eigen::MatrixXf &desired_traits_matrix,
                                                     const Eigen::MatrixXf &robot_traits_matrix) {
        Eigen::MatrixXf traits_mismatch_matrix =
                traitsMismatchMatrix(robot_traits_matrix_reduction, allocation, desired_traits_matrix,
                                     robot_traits_matrix);

        return (traits_mismatch_matrix.array() < 0).select(0, traits_mismatch_matrix);
    }

    float traitsMismatchError(const RobotTraitsMatrixReduction &robot_traits_matrix_reduction,
                              const Eigen::MatrixXf &allocation,
                              const Eigen::MatrixXf &desired_traits_matrix,
                              const Eigen::MatrixXf &robot_traits_matrix) {
        return positiveOnlyTraitsMismatchMatrix(robot_traits_matrix_reduction,
                                                allocation,
                                                desired_traits_matrix,
                                                robot_traits_matrix)
                .sum();
    }
}  // namespace grstapse