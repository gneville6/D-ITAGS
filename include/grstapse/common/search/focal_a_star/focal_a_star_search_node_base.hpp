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
#include "grstapse/common/search/a_star/a_star_search_node_base.hpp"

namespace grstapse
{
    /**!
     * A base class for search nodes for A* epsilon
     *
     * \tparam SearchNodeDeriv A derivative of FocalAStarSearchNodeBase
     */
    template <typename SearchNodeDeriv>
    class FocalAStarSearchNodeBase : public AStarSearchNodeBase<SearchNodeDeriv>
    {
        using Base = AStarSearchNodeBase<SearchNodeDeriv>;

       public:
        //! Sets the focal heuristic value
        inline void setFocalH(float h)
        {
            m_focal_h = h;
        }

        //! \returns The heuristic value representing the computation cost for the node
        [[nodiscard]] inline float focalH() const
        {
            return m_focal_h;
        }

       protected:
        FocalAStarSearchNodeBase(std::shared_ptr<SearchNodeDeriv> parent, unsigned int id)
            : Base(parent, id)
            , m_focal_h(std::nanf(""))
        {}

        float m_focal_h;
    };

}  // namespace grstapse