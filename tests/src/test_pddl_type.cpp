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
// Mocks
#include "mock_pddl_parser.hpp"
// Project
#include <grstapse/task_planning/pddl_parser/file_reader.hpp>
#include <grstapse/task_planning/pddl_parser/pddl_requirements.hpp>
#include <grstapse/task_planning/pddl_parser/pddl_symbol.hpp>
#include <grstapse/task_planning/pddl_parser/pddl_type.hpp>

namespace grstapse::unittests
{
    TEST(PddlType, empty)
    {
        FileReader reader("data/task_planning/pddl_parser/types/empty.pddl");
        mocks::MockPddlParser parser;
        parser.createPddlTask();
        parser.pddlTask()->setRequirement("typing");
        reader.checkNext(PddlSymbol::e_open_paren);
        parser.parseSingleDomainBlock(reader);
        std::shared_ptr<PddlTask> task = parser.pddlTask();
        ASSERT_EQ(task->numberOfTypes(), 0);
    }

    TEST(PddlType, single)
    {
        FileReader reader("data/task_planning/pddl_parser/types/single.pddl");
        mocks::MockPddlParser parser;
        parser.createPddlTask();
        parser.pddlTask()->setRequirement("typing");
        reader.checkNext(PddlSymbol::e_open_paren);
        parser.parseSingleDomainBlock(reader);
        std::shared_ptr<PddlTask> task = parser.pddlTask();
        ASSERT_EQ(task->numberOfTypes(), 1);
        std::shared_ptr<const PddlType> a = task->type("a");
        ASSERT_STREQ(a->name().c_str(), "a");
        ASSERT_STREQ(a->parent()->name().c_str(), "#object");
    }

    TEST(PddlType, single_parent)
    {
        FileReader reader("data/task_planning/pddl_parser/types/single_parent.pddl");
        mocks::MockPddlParser parser;
        parser.createPddlTask();
        parser.pddlTask()->setRequirement("typing");
        reader.checkNext(PddlSymbol::e_open_paren);
        parser.parseSingleDomainBlock(reader);
        std::shared_ptr<PddlTask> task = parser.pddlTask();
        ASSERT_EQ(task->numberOfTypes(), 1);

        std::shared_ptr<const PddlType> a = task->type("a");
        ASSERT_STREQ(a->name().c_str(), "a");
        ASSERT_STREQ(a->parent()->name().c_str(), "#object");
    }

    TEST(PddlType, two)
    {
        FileReader reader("data/task_planning/pddl_parser/types/two.pddl");
        mocks::MockPddlParser parser;
        parser.createPddlTask();
        parser.pddlTask()->setRequirement("typing");
        reader.checkNext(PddlSymbol::e_open_paren);
        parser.parseSingleDomainBlock(reader);
        std::shared_ptr<PddlTask> task = parser.pddlTask();
        ASSERT_EQ(task->numberOfTypes(), 2);

        std::shared_ptr<const PddlType> a = task->type("a");
        ASSERT_STREQ(a->name().c_str(), "a");
        ASSERT_STREQ(a->parent()->name().c_str(), "#object");

        std::shared_ptr<const PddlType> b = task->type("b");
        ASSERT_STREQ(b->name().c_str(), "b");
        ASSERT_STREQ(b->parent()->name().c_str(), "#object");
    }

    TEST(PddlType, two_parent)
    {
        FileReader reader("data/task_planning/pddl_parser/types/two_parent.pddl");
        mocks::MockPddlParser parser;
        parser.createPddlTask();
        parser.pddlTask()->setRequirement("typing");
        reader.checkNext(PddlSymbol::e_open_paren);
        parser.parseSingleDomainBlock(reader);
        std::shared_ptr<PddlTask> task = parser.pddlTask();
        ASSERT_EQ(task->numberOfTypes(), 2);

        std::shared_ptr<const PddlType> a = task->type("a");
        ASSERT_STREQ(a->name().c_str(), "a");
        ASSERT_STREQ(a->parent()->name().c_str(), "#object");

        std::shared_ptr<const PddlType> b = task->type("b");
        ASSERT_STREQ(b->name().c_str(), "b");
        ASSERT_EQ(b->parent(), a);
    }

    TEST(PddlType, complex)
    {
        FileReader reader("data/task_planning/pddl_parser/types/complex.pddl");
        mocks::MockPddlParser parser;
        parser.createPddlTask();
        parser.pddlTask()->setRequirement("typing");
        reader.checkNext(PddlSymbol::e_open_paren);
        parser.parseSingleDomainBlock(reader);
        std::shared_ptr<PddlTask> task = parser.pddlTask();
        ASSERT_EQ(task->numberOfTypes(), 5);

        std::shared_ptr<const PddlType> a = task->type("a");
        ASSERT_STREQ(a->name().c_str(), "a");
        ASSERT_STREQ(a->parent()->name().c_str(), "#object");

        std::shared_ptr<const PddlType> b = task->type("b");
        ASSERT_STREQ(b->name().c_str(), "b");
        ASSERT_STREQ(b->parent()->name().c_str(), "#object");

        std::shared_ptr<const PddlType> c = task->type("c");
        ASSERT_STREQ(c->name().c_str(), "c");
        ASSERT_EQ(c->parent(), a);

        std::shared_ptr<const PddlType> d = task->type("d");
        ASSERT_STREQ(d->name().c_str(), "d");
        ASSERT_EQ(d->parent(), a);

        std::shared_ptr<const PddlType> e = task->type("e");
        ASSERT_STREQ(e->name().c_str(), "e");
        ASSERT_EQ(e->parent(), b);
    }

    TEST(PddlType, unknown_parent)
    {
        FileReader reader("data/task_planning/pddl_parser/types/unknown_parent.pddl");
        mocks::MockPddlParser parser;
        parser.createPddlTask();
        parser.pddlTask()->setRequirement("typing");
        reader.checkNext(PddlSymbol::e_open_paren);
        EXPECT_ANY_THROW(parser.parseSingleDomainBlock(reader));
    }

    TEST(PddlType, typing_not_set)
    {
        FileReader reader("data/task_planning/pddl_parser/types/empty.pddl");
        mocks::MockPddlParser parser;
        parser.createPddlTask();
        reader.checkNext(PddlSymbol::e_open_paren);
        EXPECT_ANY_THROW(parser.parseSingleDomainBlock(reader));
    }
}  // namespace grstapse::unittests