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
#include <iostream>

// External
#include <robin_hood/robin_hood.hpp>

#include <boost/heap/fibonacci_heap.hpp>
// Local
#include "grstapse/common/utilities/logger.hpp"
#include "grstapse/common/utilities/mutable_priority_queue/mutable_priority_queue_comparator.hpp"
#include "grstapse/common/utilities/mutable_priority_queue/mutable_priority_queue_node.hpp"
#include "grstapse/common/utilities/mutable_priority_queue/mutable_priority_queueable.hpp"
#include "grstapse/common/utilities/noncopyable.hpp"

namespace grstapse
{
    /**!
     * \brief Wraps a mutable boost::priority_queue for easy element access
     *
     * \tparam KeyType A type used to identify the payload
     * \tparam PriorityType A type used to determine priority of the payload
     * \tparam PayloadType A type for the value of the payload
     */
    template <typename KeyType, typename PriorityType, typename PayloadType>
    requires std::derived_from<PayloadType, MutablePriorityQueueable<PriorityType>>
    class MutablePriorityQueue
    {
        using Node       = MutablePriorityQueueNode<KeyType, PriorityType, PayloadType>;
        using Comparator = MutablePriorityQueueComparator<PriorityType, Node>;
        using Heap       = boost::heap::fibonacci_heap<Node, boost::heap::compare<Comparator>>;
        using Handle     = typename Heap::handle_type;
        using Map        = robin_hood::unordered_map<KeyType, Handle>;

       public:
        using iterator         = typename Heap::iterator;
        using const_iterator   = typename Heap::const_iterator;
        using ordered_iterator = typename Heap::ordered_iterator;

        //! \returns The number of elements in the queue
        [[nodiscard]] inline size_t size() const
        {
            return m_heap.size();
        }

        //! \returns Whether there are no elements in the queue
        [[nodiscard]] inline bool empty() const
        {
            return m_heap.empty();
        }

        //! \brief Removes all elements from the queue
        void clear()
        {
            m_fast_store.clear();
            m_heap.clear();
        }

        /**!
         * Adds a new element or updates an old one
         *
         * \param key The identifier for the added element
         * \param payload The element to add
         */
        void push(const KeyType& key, const std::shared_ptr<PayloadType>& payload)
        {
            assert(payload != nullptr);

            Node node(key, payload);

            if(m_fast_store.contains(key))
            {
                Handle handle = m_fast_store[key];
                m_heap.update(handle, node);
            }
            else
            {
                Handle handle     = m_heap.push(node);
                m_fast_store[key] = handle;
            }
        }

        /**!
         * Lazy Updates an element if it is already in the queue
         * Checks if element is in the queue and if it is updates its payload without restructuring the heap
         */
        void lazyUpdate(const KeyType& key, std::shared_ptr<PayloadType> payload)
        {
            assert(payload != nullptr);
            Node node(key, payload);
            try
            {
                if(m_fast_store.contains(key))
                {
                    Handle handle = m_fast_store[key];
                    m_heap.update_lazy(handle, node);
                }
                else
                {
                    throw 20;
                }
            }
            catch(int e)
            {
                std::cout << "Node not in priority queue \n";
            }
        }

        //! \returns Whether there is a element with the associated \p key
        /**!
         * Removes an element if it is in the heap
         *
         * \p key The key of the element to erase
         */
        void erase(const KeyType& key)
        {
            if(m_fast_store.contains(key))
            {
                typename Map::iterator it = m_fast_store.find(key);
                Handle handle             = it->second;
                m_heap.erase(handle);
                m_fast_store.erase(it);
            }
            else
            {
                // todo(Andrew): Should this throw an expection?
                Logger::warn("There is no element with key {0}", key);
            }
        }

        //! \returns True if there is a element with the associated \p key, false otherwise
        [[nodiscard]] bool contains(const KeyType& key)
        {
            return m_fast_store.contains(key);
        }

        //! \returns The top element from the priority queue
        [[nodiscard]] std::shared_ptr<PayloadType> top()
        {
            assert(!m_heap.empty());
            return m_heap.top().payload();
        }

        /**!
         * Returns the top element from the priority queue and removes it from the queue
         *
         * \returns The top element from the queue
         */
        std::shared_ptr<PayloadType> pop()
        {
            assert(!m_heap.empty());
            Node node = m_heap.top();
            m_heap.pop();
            assert(m_fast_store.contains(node.key()));
            m_fast_store.erase(node.key());
            return node.payload();
        }

        /**!
         * \returns An iterator to the first element in the priority queue
         */
        [[nodiscard]] inline iterator begin()
        {
            return m_heap.begin();
        }

        /**!
         * \returns An iterator to the end of the priority queue
         */
        [[nodiscard]] inline iterator end()
        {
            return m_heap.end();
        }

        /**!
         * \returns An iterator to the first element in the priority queue
         */
        [[nodiscard]] inline const_iterator begin() const
        {
            return m_heap.begin();
        }

        /**!
         * \returns An iterator to the end of the priority queue
         */
        [[nodiscard]] inline const_iterator end() const
        {
            return m_heap.end();
        }

        /**!
         * \returns An iterator to the first element in the priority queue
         * \note Ordered iterators traverse the priority queue in heap order
         */
        [[nodiscard]] inline ordered_iterator ordered_begin() const
        {
            return m_heap.ordered_begin();
        }

        /**!
         * \returns An iterator to the end of the priority queue
         * \note Ordered iterators traverse the priority queue in heap order
         */
        [[nodiscard]] inline ordered_iterator ordered_end() const
        {
            return m_heap.ordered_begin();
        }

       private:
        Map m_fast_store;
        Heap m_heap;
    };

}  // namespace grstapse