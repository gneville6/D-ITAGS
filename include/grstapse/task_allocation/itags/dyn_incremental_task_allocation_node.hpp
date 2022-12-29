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
#include <eigen3/Eigen/Core>

// Local
#include "grstapse/common/utilities/custom_hashings.hpp"
#include "grstapse/common/utilities/matrix_dimensions.hpp"
#include "grstapse/task_allocation/itags/task_allocation_node_base.hpp"

namespace grstapse {
    //! \brief A node that contains an allocation of agents to tasks
    class DynIncrementalTaskAllocationNode : public TaskAllocationNodeBase<DynIncrementalTaskAllocationNode> {
        using Base = TaskAllocationNodeBase<DynIncrementalTaskAllocationNode>;

    public:
        /**
         * \brief Constructor for the root node
         * \param dimensions
         */
        explicit DynIncrementalTaskAllocationNode(const MatrixDimensions &dimensions);

        /**!
         * \brief Constructor for any node except the root node
         *
         * \param assignment
         * \param parent
         */
        DynIncrementalTaskAllocationNode(const Assignment &assignment,
                                         const std::shared_ptr<const DynIncrementalTaskAllocationNode> &parent);

        //! \returns A hash for this node
        [[nodiscard]] unsigned int hash() const override;

        /**!
         * \brief Setter for node ID
         *
         * \param new id
         */
        inline void setId(unsigned int id);

        /**!
         * \brief Setter for next ID
         *
         * \param next id
         */
        static inline void setNextId(unsigned int next_id);

        /**!
         * \brief Getter for next ID
         *
         * \return unsigned int next id
         */
        [[nodiscard]] static inline unsigned int nextId();

        /**!
         * \brief Getter for bool specifying if this is a dyn root
         *
         * \return unsigned int next id
         */
        [[nodiscard]] inline std::optional<bool> isDynamicRoot() const;

        /**!
         * \brief Setter for bool specifying if this is a dyn root
         *
         * \param bool is_dyn_root
         */
        inline void setIsDynamicRoot(bool is_dynamic_root);

        /**!
         * \brief Setter for matrix dimensions
         *
         * \param width
         * \param height
         */
        inline void setDimensions(int width, int height);

    private:
        std::optional<bool> m_is_dynamic_root;
    };

    void DynIncrementalTaskAllocationNode::setId(unsigned int id) {
        Base::m_id = id;
    }

    void DynIncrementalTaskAllocationNode::setNextId(unsigned int next_id) {
        Base::s_next_id = next_id;
    }

    unsigned int DynIncrementalTaskAllocationNode::nextId() {
        return Base::s_next_id;
    }

    std::optional<bool> DynIncrementalTaskAllocationNode::isDynamicRoot() const {
        return m_is_dynamic_root;
    }

    void DynIncrementalTaskAllocationNode::setIsDynamicRoot(bool is_dynamic_root) {
        m_is_dynamic_root = is_dynamic_root;
    }


    void DynIncrementalTaskAllocationNode::setDimensions(int width, int height) {
        if (m_matrix_dimensions != std::nullopt) {
            m_matrix_dimensions->height = height;
            m_matrix_dimensions->width = width;
        }
    }

}  // namespace grstapse