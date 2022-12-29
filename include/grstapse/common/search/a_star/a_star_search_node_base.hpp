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
#include "grstapse/common/search/greedy_best_first_search/greedy_best_first_search_node_base.hpp"

namespace grstapse
{
    /**!
     * Base class for a search node for an A* search
     *
     * \tparam AStarSearchNodeDeriv A derivative of AStarSearchNodeBase
     */
    template <typename AStarSearchNodeDeriv>
    class AStarSearchNodeBase : public GreedyBestFirstSearchNodeBase<AStarSearchNodeDeriv>
    {
        using Base = GreedyBestFirstSearchNodeBase<AStarSearchNodeDeriv>;

       public:
        //! Sets the path cost for a node
        inline void setG(float g)
        {
            m_g = g;
        }

        //! \returns The path cost of a node
        [[nodiscard]] inline float g() const
        {
            return m_g;
        }

        //! \returns The total valuation of a node
        [[nodiscard]] inline float f() const final override
        {
            return Base::m_h + m_g;
        }

       protected:
        /**!
         * \brief Constructor
         *
         * \param id A unique identifier for this node
         * \param parent The parent of this AStarSearchNodeDeriv
         */
        AStarSearchNodeBase(const unsigned int id, const std::shared_ptr<const AStarSearchNodeDeriv>& parent = nullptr)
            : Base(id, parent)
            , m_g(std::nanf(""))
        {}

        float m_g;
    };

    /**!
     * Concept to force a type to derive from AStarSearchNodeBase
     *
     * \tparam T
     */
    template <typename T>
    concept AStarSearchNodeDeriv = std::derived_from<T, AStarSearchNodeBase<T>>;
}  // namespace grstapse