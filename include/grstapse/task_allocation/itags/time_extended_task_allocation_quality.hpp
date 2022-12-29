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

// Local
#include "grstapse/common/search/heuristic_base.hpp"
#include "grstapse/task_allocation/itags/allocation_percentage_remaining.hpp"
#include "grstapse/task_allocation/itags/incremental_task_allocation_node.hpp"
#include "grstapse/task_allocation/itags/normalized_schedule_quality.hpp"
#include "grstapse/task_allocation/itags/task_allocation_node_base.hpp"

namespace grstapse
{
    class ItagsProblemInputs;

    /**!
     * \brief Computes the Time Extended Task Allocation Quality heuristic
     *
     * This heuristic is a convex combination of Allocation Percentage Remaining and Normalized Schedule Quality
     *
     * \see AllocationPercentageRemaining
     * \see NormalizedScheduleQuality
     * \see Itags
     *
     * \tparam NodeDeriv The node type that tetaq will be calculated on
     *
     * \cite Neville, G., Messing , A., Ravichandar, H., Hutchinson, S.,& Chernova, S. (n.d.). IEEE/RSJ International
     Conference on Intelligent Robots and Systems (IROS 2021) . In An Interleaved Approach to Trait-Based Task
     Allocation and Scheduling.

     */
    template <typename NodeDeriv = IncrementalTaskAllocationNode>
    requires std::derived_from<NodeDeriv, TaskAllocationNodeBase<NodeDeriv>>
    class TimeExtendedTaskAllocationQuality : public HeuristicBase<NodeDeriv>
    {
       public:
        //! \brief Constructor
        explicit TimeExtendedTaskAllocationQuality(const std::shared_ptr<const ItagsProblemInputs> &problem_inputs)
            : m_apr(problem_inputs)
            , m_nsq(problem_inputs)
        {
            m_alpha = problem_inputs->m_alpha;
        }

        //! \returns A combination of APR and NSQ heuristics
        [[nodiscard]] float operator()(const std::shared_ptr<NodeDeriv> &node) const final
        {
            return m_alpha * getAPR(node) + (1.0f - m_alpha) * getNSQ(node);
        }

        //! \returns A the value for the APR heuristic
        inline float getAPR(const std::shared_ptr<NodeDeriv> &node) const
        {
            float apr = m_apr(node);
            return apr;
        }

        //! \returns A the value for the NSQ heuristic
        inline float getNSQ(const std::shared_ptr<NodeDeriv> &node) const
        {
            float nsq = m_nsq(node);
            return nsq;
        }

        //! \returns A the value for the alpha
        [[nodiscard]] inline float getAlpha() const
        {
            return m_alpha;
        }

       private:
        AllocationPercentageRemaining<NodeDeriv> m_apr;
        NormalizedScheduleQuality<NodeDeriv> m_nsq;
        float m_alpha;
    };
}  // namespace grstapse