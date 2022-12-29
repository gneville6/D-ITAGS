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

// External
//// eigen
#include <iostream>

#include <eigen3/Eigen/Core>

// Local
#include "grstapse/common/search/greedy_best_first_search/greedy_best_first_search_node_base.hpp"
#include "grstapse/common/utilities/custom_hashings.hpp"
#include "grstapse/common/utilities/matrix_dimensions.hpp"
#include "grstapse/task_allocation/assignment.hpp"

namespace grstapse {
    // Forward Declaration
    class ScheduleBase;

    /**!
     * \brief A node that contains an allocation of agents to tasks used as a base class for other allocation nodes
     *
     * \tparam NodeDeriv The node type that apr will be calculated on
     *
     *
     */
    template<typename NodeDeriv>
    class TaskAllocationNodeBase : public GreedyBestFirstSearchNodeBase<NodeDeriv> {
        using Base = GreedyBestFirstSearchNodeBase<NodeDeriv>;

    public:
        /**
         * \brief Constructor for the root node
         * \param dimensions
         */
        TaskAllocationNodeBase(const MatrixDimensions &dimensions)
                : Base(s_next_id++, nullptr), m_last_assigment(std::nullopt), m_matrix_dimensions(dimensions),
                  m_schedule(nullptr) {}

        /**!
         * \brief Constructor for any node except the root node
         *
         * \param matrix_cell
         * \param parent
         */
        TaskAllocationNodeBase(const Assignment &assignment, const std::shared_ptr<const NodeDeriv> &parent)
                : Base(s_next_id++, parent), m_last_assigment(assignment), m_matrix_dimensions(std::nullopt),
                  m_schedule(nullptr) {
            assert(parent);
        }

        //! \returns The last assigment (robot_id, task_id)
        [[nodiscard]] inline const std::optional<Assignment> &lastAssigment() const {
            return m_last_assigment;
        }

        //! \returns The dimensions of the allocation matrix
        [[nodiscard]] const MatrixDimensions &matrixDimensions() const {
            if (m_matrix_dimensions.has_value()) {
                return m_matrix_dimensions.value();
            }
            return Base::m_parent->matrixDimensions();
        }

        /**!
         * \returns The allocation contained by this node
         *
         * \note Virtual for unit tests
         */
        virtual Eigen::MatrixXf allocation() const {
            const MatrixDimensions &dimensions = matrixDimensions();
            Eigen::MatrixXf matrix(dimensions.height, dimensions.width);  // (rows, columns)
            matrix.setZero();
            if (m_last_assigment == std::nullopt) {
                return matrix;
            }

            matrix(m_last_assigment->task, m_last_assigment->robot) = 1.0f;

            std::shared_ptr<const NodeDeriv> parent;
            for (parent = Base::m_parent; parent->m_last_assigment != std::nullopt; parent = parent->parent()) {
                matrix(parent->m_last_assigment->task, parent->m_last_assigment->robot) = 1.0f;
            }
            return matrix;
        }

        //! \returns A hash for this node
        [[nodiscard]] unsigned int hash() const override {
            return std::hash<Eigen::MatrixXf>()(allocation());
        }

        //! \returns A hash for this node
        std::shared_ptr<const NodeDeriv> getParent() const {
            return Base::m_parent;
        }

        /**!
         * \brief Getter for APR Heuristic
         *
         * \return optional float representing heuristic value
         */
        [[nodiscard]] inline std::optional<float> getAPR() const {
            return m_apr;
        }

        /**!
         * \brief Getter NSQ Heuristic
         *
         * \return optional float representing heuristic value
         */
        [[nodiscard]] inline std::optional<float> getNSQ() const {
            return m_nsq;
        }

        /**!
         * \brief Setter for APR Heuristic
         *
         * \param float for new APR value
         */
        inline void setAPR(std::optional<float> APR) {
            m_apr = APR;
        }

        /**!
         * \brief Getter NSQ Heuristic
         *
         * \param float for new NSQ value
         */
        inline void setNSQ(std::optional<float> NSQ) {
            m_nsq = NSQ;
        }

        //! \returns The schedule computed for this node if NSQ was run
        [[nodiscard]] inline const std::shared_ptr<const ScheduleBase> &schedule() const {
            return m_schedule;
        }

        std::shared_ptr<const ScheduleBase> m_schedule;

    protected:
        std::optional<Assignment> m_last_assigment;
        std::optional<MatrixDimensions> m_matrix_dimensions;
        static unsigned int s_next_id;
        std::optional<float> m_apr;
        std::optional<float> m_nsq;

    };

}  // namespace grstapse
