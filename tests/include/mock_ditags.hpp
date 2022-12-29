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
#include <Eigen/Core>

// Project
#include <grstapse/task_allocation/itags/ditags_tetaq.hpp>
#include <grstapse/task_allocation/itags/dyn_incremental_task_allocation_node.hpp>

namespace grstapse::mocks
{
    // Forward Declarations

    /**!
     * Mock which replaces the allocation function
     *
     * \see IncrementalTaskAllocationNode
     */
    class MockDitags : public DitagsTetaq
    {
       public:
        using Base = DitagsTetaq;

        /**!
         * \brief Constructor
         *
         * @param parameters input problems
         */
        MockDitags(const std::shared_ptr<const ItagsProblemInputs>& problem_inputs);

        /**!
         * \brief Shallow copy just open set
         *
         * @param parameters input problems
         */
        MockDitags(const DitagsTetaq& to_copy, bool is_deep_copy = true);

        MutablePriorityQueue<unsigned int, float, DynIncrementalTaskAllocationNode>& getOpen();

        std::vector<std::shared_ptr<DynIncrementalTaskAllocationNode>>& getClosed();

        std::set<unsigned int>& getClosedID();

        std::vector<std::shared_ptr<DynIncrementalTaskAllocationNode>>& getPruned();

        std::set<unsigned int>& getPrunedID();

        std::shared_ptr<const ItagsProblemInputs>& getItagsProblemInputs();

        float recalcAPR(std::shared_ptr<DynIncrementalTaskAllocationNode> node);

        float recalcNSQ(std::shared_ptr<DynIncrementalTaskAllocationNode> node);

        std::shared_ptr<DynIncrementalTaskAllocationNode> popOpen();

        void addNodeToOpen(std::shared_ptr<DynIncrementalTaskAllocationNode> node);

        void addNodeToClosed(std::shared_ptr<DynIncrementalTaskAllocationNode> node);

        void addNodeToPruned(std::shared_ptr<DynIncrementalTaskAllocationNode> node);

        /**!
         * \brief Setter for bool defining if schedules are stale
         *
         * @param bool is_stale
         */
        inline void setClosedStaleNSQ(bool is_stale);

        /**!
         * \brief Getter for bool defining if schedules are stale
         *
         * @return bool is_stale
         */
        [[nodiscard]] inline bool getClosedStaleNSQ() const;

        /**!
         * \brief Setter for bool defining if APR values are stale
         *
         * @param bool is_stale
         */
        inline void setClosedStaleAPR(bool is_stale);
        /**!
         * \brief Getter for bool defining if APR values are stale
         *
         * @return bool is_stale
         */
        [[nodiscard]] inline bool getClosedStaleAPR() const;

        /**!
         * \brief Setter for bool defining if schedules are stale
         *
         * @param bool is_stale
         */
        inline void setPrunedStaleNSQ(bool is_stale);

        /**!
         * \brief Getter for bool defining if schedules are stale
         *
         * @return bool is_stale
         */
        [[nodiscard]] inline bool getPrunedStaleNSQ() const;

        /**!
         * \brief Setter for bool defining if APR values are stale
         *
         * @param bool is_stale
         */
        inline void setPrunedStaleAPR(bool is_stale);

        /**!
         * \brief Getter for bool defining if APR values are stale
         *
         * @return bool is_stale
         */
        [[nodiscard]] inline bool getPrunedStaleAPR() const;

        using DitagsTetaq::addNewNodesFromRoot;
        using DitagsTetaq::addPreviousSolutionToOpen;
        using DitagsTetaq::agentsLost;
        using DitagsTetaq::needToUpdateClosedAPR;
        using DitagsTetaq::needToUpdateOpenAPR;
        using DitagsTetaq::needToUpdateOpenNSQ;
        using DitagsTetaq::needToUpdatePrunedAPR;
        using DitagsTetaq::recomputeTETAQLocal;
        using DitagsTetaq::shouldRemoveLostAgent;
        using DitagsTetaq::updateForLostAgent;
        using DitagsTetaq::updateFunctors;
        using DitagsTetaq::updateNodeAPR;
        using DitagsTetaq::updateNodeNSQ;
        using DitagsTetaq::updateNodesClosedAPR;
        using DitagsTetaq::updateNodesOpenAPR;
        using DitagsTetaq::updateNodesOpenNSQ;
        using DitagsTetaq::updateNodesPrunedAPR;
        using DitagsTetaq::wasAgentLost;
        using DitagsTetaq::wasNewAgentAdded;

        // not implement yet
        using DitagsTetaq::motionPlansNeedUpdate;
        using DitagsTetaq::updateMotionPlanningMap;
    };

    inline void MockDitags::setClosedStaleNSQ(bool is_stale)
    {
        Base::m_is_closed_nsq_stale = is_stale;
    }

    [[nodiscard]] inline bool MockDitags::getClosedStaleNSQ() const
    {
        return m_is_closed_nsq_stale;
    }

    inline void MockDitags::setClosedStaleAPR(bool is_stale)
    {
        m_is_closed_apr_stale = is_stale;
    }

    [[nodiscard]] inline bool MockDitags::getClosedStaleAPR() const
    {
        return m_is_closed_apr_stale;
    }

    inline void MockDitags::setPrunedStaleNSQ(bool is_stale)
    {
        m_is_pruned_nsq_stale = is_stale;
    }

    [[nodiscard]] inline bool MockDitags::getPrunedStaleNSQ() const
    {
        return m_is_pruned_nsq_stale;
    }

    inline void MockDitags::setPrunedStaleAPR(bool is_stale)
    {
        m_is_pruned_apr_stale = is_stale;
    }

    [[nodiscard]] inline bool MockDitags::getPrunedStaleAPR() const
    {
        return m_is_pruned_apr_stale;
    }

}  // namespace grstapse::mocks