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
#include <cassert>
#include <concepts>
#include <memory>
#include <vector>

// Local
#include "grstapse/common/search/edge_applier_base.hpp"
#include "grstapse/common/search/search_node_base.hpp"
#include "grstapse/common/utilities/noncopyable.hpp"

namespace grstapse
{
    /**!
     * \brief The abstract base class for all classes that generate the successors of a node
     *
     * \tparam SearchNodeDeriv A derivative of SearchNodeBase
     */
    template <SearchNodeDeriv SearchNode>
    class SuccessorGeneratorBase : private Noncopyable
    {
       protected:
        using EdgeApplier = EdgeApplierBase<SearchNode>;

       public:
        //! \brief Default Constructor
        SuccessorGeneratorBase() = default;

        /**!
         * \brief A constructor which sets the list of possible edge appliers
         */
        explicit SuccessorGeneratorBase(const std::vector<std::shared_ptr<const EdgeApplier>>& edge_appliers)
            : m_edge_appliers(edge_appliers)
        {}

        /**!
         * \brief Sets the list of possible edge appliers
         */
        inline void setEdgeAppliers(const std::vector<std::shared_ptr<const EdgeApplier>>& edge_appliers)
        {
            m_edge_appliers = edge_appliers;
        }

        //! \returns A list of the successors of a node
        [[nodiscard]] std::vector<std::shared_ptr<SearchNode>> operator()(const std::shared_ptr<SearchNode>& base) const
        {
            std::vector<std::shared_ptr<SearchNode>> rv;
            std::shared_ptr<SearchNode> node;
            for(std::shared_ptr<const EdgeApplier> edge_applier: m_edge_appliers)
            {
                if(!edge_applier->isApplicable(base))
                {
                    continue;
                }

                node = edge_applier->apply(base);
                assert(node);

                if(isValidNode(node))
                {
                    rv.push_back(node);
                }
            }
            return rv;
        }

       protected:
        //! \returns Whether \p is a valid node
        [[nodiscard]] virtual bool isValidNode(const std::shared_ptr<const SearchNode>& node) const = 0;

        std::vector<std::shared_ptr<const EdgeApplier>> m_edge_appliers;
    };
}  // namespace grstapse