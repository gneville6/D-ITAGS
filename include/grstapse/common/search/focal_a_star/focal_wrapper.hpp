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
#include <concepts>

// Local
#include "grstapse/common/search/focal_a_star/focal_a_star_search_node_base.hpp"
#include "grstapse/common/utilities/mutable_priority_queue/mutable_priority_queueable.hpp"

namespace grstapse
{
    /**!
     * \brief Wraps a A* epsilon search node for the focal search
     *
     * \tparam SearchNodeDerive
     */
    template <typename SearchNodeDeriv>
    requires std::derived_from<SearchNodeDeriv, FocalAStarSearchNodeBase<SearchNodeDeriv>>
    class FocalWrapper : public MutablePriorityQueueable<float>
    {
       public:
        FocalWrapper(const std::shared_ptr<SearchNodeDeriv>& internal)
            : m_internal(internal)
        {}

        [[nodiscard]] inline const std::shared_ptr<SearchNodeDeriv>& internal() const
        {
            return m_internal;
        }

        [[nodiscard]] float priority() const override
        {
            return m_internal->focalH();
        }

       private:
        std::shared_ptr<SearchNodeDeriv> m_internal;
    };

}  // namespace grstapse