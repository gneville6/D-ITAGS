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
#include <algorithm>
#include <memory>
#include <vector>

#include <type_traits>
// Local
#include "grstapse/common/utilities/noncopyable.hpp"

namespace grstapse
{
    /**!
     * Marks the status of a node
     */
    enum class SearchNodeStatus : uint8_t
    {
        e_new = 0,
        e_open,
        e_closed,
        e_deadend,
        e_pruned
    };

    /**!
     * \brief A node in a graph or tree search (DFS, A*, etc)
     *
     * \tparam SearchNodeDeriv A derivative of SearchNodeBase
     */
    template <typename SearchNodeDeriv>
    class SearchNodeBase : private Noncopyable
    {
       public:
        //! \returns The parent of this node
        [[nodiscard]] inline const std::shared_ptr<const SearchNodeDeriv>& parent() const
        {
            return m_parent;
        }

        //! \brief Sets the status of this node
        inline void setStatus(SearchNodeStatus status)
        {
            m_status = status;
        }

        //! \returns The status of this node
        [[nodiscard]] inline SearchNodeStatus status() const
        {
            return m_status;
        }

        //! \returns A unique identifier for this node
        [[nodiscard]] inline unsigned int id() const
        {
            return m_id;
        }

        //! \returns The hash identifier for this node
        [[nodiscard]] virtual unsigned int hash() const = 0;

       protected:
        /**!
         * \brief Constructor
         *
         * \param id A unique identifier for this node
         * \param parent The parent of this SearchNodeDerive
         */
        SearchNodeBase(const unsigned int id, const std::shared_ptr<const SearchNodeDeriv>& parent = nullptr)
            : m_id(id)
            , m_parent(parent)
            , m_status(SearchNodeStatus::e_new)
        {}

        unsigned int m_id;
        std::shared_ptr<const SearchNodeDeriv> m_parent;
        SearchNodeStatus m_status;
    };

    /**!
     * Concept to force a type to derive from SearchNodeBase
     *
     * \tparam T A derivative of SearchNodeBase
     */
    template <typename T>
    concept SearchNodeDeriv = std::derived_from<T, SearchNodeBase<T>>;

    /**!
     * Traces a node back to its root and creates a vector of the path from root to \p node
     *
     * \tparam SearchNode A derivative of SearchNodeBase
     */
    template <SearchNodeDeriv SearchNode>
    std::vector<std::shared_ptr<const SearchNode>> trace(std::shared_ptr<const SearchNode> node)
    {
        std::vector<std::shared_ptr<const SearchNode>> rv;
        for(; node != nullptr; node = node->parent())
        {
            rv.push_back(node);
        }
        std::reverse(rv.begin(), rv.end());
        return rv;
    }

    /**!
     * Traces a node back to its root and applies a function to each node
     *
     * \tparam SearchNode A derivative of SearchNodeBase
     */
    template <SearchNodeDeriv SearchNode>
    void traceApply(std::shared_ptr<const SearchNode> node,
                    const std::function<void(const std::shared_ptr<const SearchNode>&)>& function)
    {
        for(; node != nullptr; node = node->parent())
        {
            function(node);
        }
    }
}  // namespace grstapse