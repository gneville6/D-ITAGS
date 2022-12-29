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
// External
#include <gtest/gtest.h>
// Project
#include <grstapse/common/utilities/mutable_priority_queue/mutable_priority_queue.hpp>

namespace grstapse::unittests
{
    class TestDummy : public MutablePriorityQueueable<int>
    {
       public:
        TestDummy(int value)
            : m_value(value)
        {}

        int value() const
        {
            return m_value;
        }

        int priority() const override
        {
            return m_value;
        }

       private:
        int m_value;
    };

    TEST(MutablePriorityQueue, Basic)
    {
        MutablePriorityQueue<int, int, TestDummy> queue;

        for(int i = 0; i < 10; ++i)
        {
            queue.push(i, std::make_shared<TestDummy>(i));
        }

        ASSERT_EQ(queue.size(), 10);
        for(int i = 0; i < 10; ++i)
        {
            std::shared_ptr<TestDummy> dummy = queue.pop();
            ASSERT_EQ(dummy->value(), i);
        }
        ASSERT_TRUE(queue.empty());
    }

    TEST(MutablePriorityQueue, DifferentKey)
    {
        MutablePriorityQueue<int, int, TestDummy> queue;

        for(int i = 0; i < 10; ++i)
        {
            queue.push(9 - i, std::make_shared<TestDummy>(i));
        }

        for(int i = 0; i < 10; ++i)
        {
            std::shared_ptr<TestDummy> dummy = queue.pop();
            ASSERT_EQ(dummy->value(), i);
        }
    }

    TEST(MutablePriorityQueue, Contains)
    {
        MutablePriorityQueue<int, int, TestDummy> queue;

        for(int i = 0; i < 10; ++i)
        {
            queue.push(9 - i, std::make_shared<TestDummy>(i));
        }

        ASSERT_TRUE(queue.contains(5));
    }

    TEST(MutablePriorityQueue, erase)
    {
        MutablePriorityQueue<int, int, TestDummy> queue;

        for(int i = 0; i < 10; ++i)
        {
            queue.push(i, std::make_shared<TestDummy>(i));
        }

        ASSERT_TRUE(queue.contains(5));
        ASSERT_TRUE(queue.size() == 10);
        queue.erase(5);
        ASSERT_FALSE(queue.contains(5));
        ASSERT_TRUE(queue.size() == 9);
        for(int i = 0; i < 10; ++i)
        {
            if(i != 5)
            {
                ASSERT_TRUE(queue.contains(i));
            }
        }

        for(int i = 0; i < 3; ++i)
        {
            queue.erase(i);
        }
        std::shared_ptr<TestDummy> test_dummy = queue.pop();
        ASSERT_TRUE(test_dummy->value() == 3);
    }
}  // namespace grstapse::unittests