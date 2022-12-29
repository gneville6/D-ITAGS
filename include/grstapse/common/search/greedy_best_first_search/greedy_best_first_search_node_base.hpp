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
#include <cmath>

// Local
#include "grstapse/common/search/best_first_search_node_base.hpp"

namespace grstapse
{
    /**!
     * Base class for a search node for a greedy best first search
     *
     * \tparam GreedyBestFirstSearchNodeDeriv A derivative of GreedyBestFirstSearchNodeDeriv
     */
    template <typename GreedyBestFirstSearchNodeDeriv>
    class GreedyBestFirstSearchNodeBase : public BestFirstSearchNodeBase<GreedyBestFirstSearchNodeDeriv>
    {
        using Base = BestFirstSearchNodeBase<GreedyBestFirstSearchNodeDeriv>;
       public:
        //! \brief Sets the heuristic value of this node (distance to the goal)
        inline void setH(float h)
        {
            m_h = h;
        }

        //! \returns The heuristic value of this node (distance to the goal)
        [[nodiscard]] inline float h() const
        {
            return m_h;
        }

        //! \returns The cost of this node
        [[nodiscard]] inline float f() const override
        {
            return m_h;
        }

       protected:
        /**!
         * \brief Constructor
         *
         * \param id A unique identifier for this node
         * \param parent The parent of this GreedyBestFirstSearchNodeDeriv
         */
        GreedyBestFirstSearchNodeBase(const unsigned int id,
                                      const std::shared_ptr<const GreedyBestFirstSearchNodeDeriv>& parent = nullptr)
            : Base(id, parent)
            , m_h(std::nanf(""))
        {}

        float m_h;
    };

    /**!
     * Concept to force a type to derive from GreedyBestFirstSearchNodeBase
     *
     * \tparam T
     */
    template <typename T>
    concept GreedyBestFirstSearchNodeDeriv = std::derived_from<T, GreedyBestFirstSearchNodeBase<T>>;
}  // namespace grstapse