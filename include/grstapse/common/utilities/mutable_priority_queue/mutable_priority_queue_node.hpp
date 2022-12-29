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
#include "mutable_priority_queueable.hpp"

namespace grstapse
{
    template <typename KeyType, typename PriorityType, typename PayloadType>
    requires std::derived_from<PayloadType, MutablePriorityQueueable<PriorityType>>

    class MutablePriorityQueueNode : public MutablePriorityQueueable<PriorityType>
    {
       public:
        MutablePriorityQueueNode(const KeyType& key, std::shared_ptr<PayloadType> payload)
            : m_key(key)
            , m_payload(payload)
        {}

        [[nodiscard]] inline const KeyType& key() const
        {
            return m_key;
        }

        [[nodiscard]] inline std::shared_ptr<PayloadType> payload()
        {
            return m_payload;
        }

        [[nodiscard]] inline std::shared_ptr<PayloadType> payload() const
        {
            return m_payload;
        }

        [[nodiscard]] inline PriorityType priority() const
        {
            return m_payload->priority();
        }

       private:
        KeyType m_key;
        std::shared_ptr<PayloadType> m_payload;
    };

}  // namespace grstapse