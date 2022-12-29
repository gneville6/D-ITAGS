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
#include <grstapse/core.hpp>
// Local
#include "mock_pddl_parser.hpp"

namespace grstapse::unittests
{
    TEST(PddlObject, empty_constants)
    {
        FileReader reader("data/task_planning/pddl_parser/objects/empty_constants.pddl");
        mocks::MockPddlParser parser;
        parser.createPddlTask();
        parser.pddlTask()->setRequirement("typing");
        reader.checkNext(PddlSymbol::e_open_paren);
        parser.parseSingleDomainBlock(reader);
        std::shared_ptr<PddlTask> task = parser.pddlTask();
        ASSERT_EQ(task->totalNumberOfObjects(), 0);
    }

    TEST(PddlObject, single_type_constants)
    {
        FileReader reader("data/task_planning/pddl_parser/objects/single_type_constants.pddl");
        mocks::MockPddlParser parser;
        parser.createPddlTask();
        std::shared_ptr<PddlTask> task = parser.pddlTask();
        task->setRequirement("typing");
        task->addType("c", "#object");
        reader.checkNext(PddlSymbol::e_open_paren);
        parser.parseSingleDomainBlock(reader);
        ASSERT_EQ(task->totalNumberOfObjects(), 2);

        std::shared_ptr<const PddlType> c = task->type("c");

        std::shared_ptr<const PddlObject> a = task->object("a");
        ASSERT_STREQ(a->name().c_str(), "a");
        ASSERT_TRUE(a->constant());
        ASSERT_EQ(a->type(), c);

        std::shared_ptr<const PddlObject> b = task->object("b");
        ASSERT_STREQ(b->name().c_str(), "b");
        ASSERT_TRUE(b->constant());
        ASSERT_EQ(b->type(), c);
    }

    TEST(PddlObject, multiple_types_constants)
    {
        FileReader reader("data/task_planning/pddl_parser/objects/multiple_type_constants.pddl");
        mocks::MockPddlParser parser;
        parser.createPddlTask();
        std::shared_ptr<PddlTask> task = parser.pddlTask();
        task->setRequirement("typing");
        task->addType("c", "#object");
        task->addType("d", "#object");

        reader.checkNext(PddlSymbol::e_open_paren);
        parser.parseSingleDomainBlock(reader);
        ASSERT_EQ(task->totalNumberOfObjects(), 2);

        std::shared_ptr<const PddlType> c = task->type("c");
        std::shared_ptr<const PddlType> d = task->type("d");

        std::shared_ptr<const PddlObject> a = task->object("a");
        ASSERT_STREQ(a->name().c_str(), "a");
        ASSERT_TRUE(a->constant());
        ASSERT_EQ(a->type(), c);

        std::shared_ptr<const PddlObject> b = task->object("b");
        ASSERT_STREQ(b->name().c_str(), "b");
        ASSERT_TRUE(b->constant());
        ASSERT_EQ(b->type(), d);
    }

    TEST(PddlObject, empty_objects)
    {
        // todo
    }

    TEST(PddlObject, single_type_objects)
    {
        // todo
    }

    TEST(PddlObject, multiple_types_objects)
    {
        // todo
    }
}  // namespace grstapse::unittests