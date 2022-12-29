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
#include <memory>
#include <vector>

// External
#include <robin_hood/robin_hood.hpp>

// Local
#include "grstapse/common/search/search_node_base.hpp"
#include "grstapse/common/utilities/mutable_priority_queue/mutable_priority_queueable.hpp"

namespace grstapse
{
    // Forward Declarations
    class ConflictBase;
    class ConstraintBase;
    class ConstraintTreeNode;
    class GridCell;
    class GridMap;
    class TemporalGridCellNode;

    //! Specifies the type of cost that should be calculated
    enum class ConstraintTreeNodeCostType : uint8_t
    {
        e_makespan = 0,
        e_sum_of_costs
    };

    /**!
     * Abstract base class for a constraint tree node (Used to split root from the rest)
     *
     * \see ConstraintTreeNode
     * \see ConstraintTreeNodeRoot
     */
    class ConstraintTreeNodeBase
        : public SearchNodeBase<ConstraintTreeNodeBase>
        , public MutablePriorityQueueable<unsigned int>
    {
       public:
        /**!
         * \brief Constructor
         *
         * \param num_robots The number of robots
         * \param cost The type of cost used
         * \param parent The parent node
         */
        ConstraintTreeNodeBase(unsigned int num_robots,
                               ConstraintTreeNodeCostType cost,
                               const std::shared_ptr<const ConstraintTreeNodeBase>& parent);

        //! \brief Sets a low level trajectory
        virtual void setLowLevelSolution(unsigned int robot,
                                         const std::shared_ptr<const TemporalGridCellNode>& leaf) = 0;

        //! \returns A low level trajectory for \p robot
        [[nodiscard]] virtual const std::vector<std::shared_ptr<const TemporalGridCellNode>>& lowLevelSolution(
            unsigned int robot) const = 0;

        //! \brief Sets a constraint for \p robot
        virtual void setConstraint(unsigned int robot, const std::shared_ptr<const ConstraintBase>& constraint) = 0;

        //! \returns A set of constraints for \p robot
        [[nodiscard]] virtual const robin_hood::unordered_set<std::shared_ptr<const ConstraintBase>> constraints(
            unsigned int robot) const = 0;

        //! \returns The cost of this node
        [[nodiscard]] unsigned int cost() const;

        //! \returns The maximum duration of the lower level solutions
        [[nodiscard]] unsigned int makespan() const;

        //! \returns The sum of the durations of the lower level solutions
        [[nodiscard]] unsigned int sumOfCosts() const;

        //! \returns The first conflict in the lower level solutions (with respect to time and then vertex before edge
        //! conflicts)
        [[nodiscard]] std::unique_ptr<const ConflictBase> getFirstConflict() const;

        //! \copydoc SearchNodeBase
        [[nodiscard]] inline unsigned int hash() const final override;

        //! \copydoc MutablePriorityQueueable
        [[nodiscard]] unsigned int priority() const final override;

        void display(const std::shared_ptr<const GridMap>& map) const;

       protected:
        //! \returns The position of a specific \p robot at a specific \p time including if it has finish
        [[nodiscard]] std::shared_ptr<const GridCell> getStateOrLast(unsigned int robot, unsigned int time) const;

        //! \returns The position of a specific \p robot at a specific \p time or nullptr if it has finished
        [[nodiscard]] std::shared_ptr<const GridCell> getState(unsigned int robot, unsigned int time) const;

        //! \returns The position of a specific \p robot at the end of its route
        [[nodiscard]] std::shared_ptr<const GridCell> getLastState(unsigned int robot) const;

        virtual void constraintsInsert(unsigned int robot,
                                       robin_hood::unordered_set<std::shared_ptr<const ConstraintBase>>& us) const = 0;

        unsigned int m_num_robots;
        ConstraintTreeNodeCostType m_cost_type;

        static unsigned int s_next_id;

        friend class ConstraintTreeNode;
    };

    // Inline functions
    unsigned int ConstraintTreeNodeBase::hash() const
    {
        return m_id;
    }

}  // namespace grstapse