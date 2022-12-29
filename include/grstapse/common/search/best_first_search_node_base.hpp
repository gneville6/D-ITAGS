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

// Local
#include "grstapse/common/search/search_node_base.hpp"
#include "grstapse/common/utilities/mutable_priority_queue/mutable_priority_queueable.hpp"

namespace grstapse
{
    /**!
     * Base class for a search node for a best first search
     *
     * \tparam BestFirstSearchNodeDeriv A derivative of BestFirstSearchNodeBase
     */
    template <typename BestFirstSearchNodeDeriv>
    class BestFirstSearchNodeBase
        : public SearchNodeBase<BestFirstSearchNodeDeriv>
        , public MutablePriorityQueueable<float>
    {
        using Base = SearchNodeBase<BestFirstSearchNodeDeriv>;

       public:
        //! \returns The cost of a node
        [[nodiscard]] virtual float f() const = 0;

        /**!
         * \returns The priority for MutablePriorityQueue
         *
         * \see MutablePriorityQueue
         * \see MutablePriorityQueueable
         */
        [[nodiscard]] inline float priority() const override
        {
            return f();
        }

       protected:
        /**!
         * \brief Constructor
         *
         * \param id A unique identifier for this node
         * \param parent The parent of this BestFirstSearchNodeDeriv
         */
        BestFirstSearchNodeBase(const unsigned int id,
                                const std::shared_ptr<const BestFirstSearchNodeDeriv>& parent = nullptr)
            : Base(id, parent)
        {}
    };

    /**!
     * Concept to force a type to derive from BestFirstSearchNodeBase
     *
     * \tparam T
     */
    template <typename T>
    concept BestFirstSearchNodeDeriv = std::derived_from<T, BestFirstSearchNodeBase<T>>;
}  // namespace grstapse