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
//// googletest
#include <gtest/gtest.h>

// Mocks
#include "mock_pddl_parser.hpp"
// Project
#include <grstapse/task_planning/pddl_parser/file_reader.hpp>
#include <grstapse/task_planning/pddl_parser/pddl_function.hpp>
#include <grstapse/task_planning/pddl_parser/pddl_requirements.hpp>
#include <grstapse/task_planning/pddl_parser/pddl_symbol.hpp>
#include <grstapse/task_planning/pddl_parser/pddl_type.hpp>
#include <grstapse/task_planning/pddl_parser/pddl_variable.hpp>

namespace grstapse::unittests
{
    /**!
     * Tests parsing a predicates block with no predicates
     */
    TEST(PddlFunction, empty_predicates)
    {
        FileReader reader("data/task_planning/pddl_parser/functions/predicates/empty.pddl");
        mocks::MockPddlParser parser;
        parser.createPddlTask();
        parser.pddlTask()->setRequirement("typing");
        reader.checkNext(PddlSymbol::e_open_paren);
        parser.parseSingleDomainBlock(reader);
        std::shared_ptr<PddlTask> task = parser.pddlTask();
        ASSERT_EQ(task->totalNumberOfFunctions(), 0);
    }

    /**!
     * Tests parsing a predicates block with a single predicate with no parameters
     */
    TEST(PddlFunction, single_predicate_no_parameters)
    {
        FileReader reader("data/task_planning/pddl_parser/functions/predicates/single_predicate_no_parameters.pddl");
        mocks::MockPddlParser parser;
        parser.createPddlTask();
        parser.pddlTask()->setRequirement("typing");
        reader.checkNext(PddlSymbol::e_open_paren);
        parser.parseSingleDomainBlock(reader);
        std::shared_ptr<PddlTask> task = parser.pddlTask();
        ASSERT_EQ(task->numberOfPredicates(), 1);
        ASSERT_EQ(task->totalNumberOfFunctions(), 1);

        std::shared_ptr<const PddlFunction> a = task->function("a");
        ASSERT_EQ(a->parameters().size(), 0);
    }

    /**!
     * Tests parsing a predicates block with a single predicate with multiple parameters of a single type
     */
    TEST(PddlFunction, single_predicate_single_type_parameters)
    {
        FileReader reader(
            "data/task_planning/pddl_parser/functions/predicates/single_predicate_single_type_parameters.pddl");
        mocks::MockPddlParser parser;
        parser.createPddlTask();
        parser.pddlTask()->setRequirement("typing");
        std::shared_ptr<PddlTask> task = parser.pddlTask();
        task->addType("d", "object");

        reader.checkNext(PddlSymbol::e_open_paren);
        parser.parseSingleDomainBlock(reader);
        ASSERT_EQ(task->numberOfPredicates(), 1);
        ASSERT_EQ(task->totalNumberOfFunctions(), 1);

        std::shared_ptr<const PddlFunction> a = task->function("a");
        ASSERT_EQ(a->parameters().size(), 2);

        const std::vector<PddlVariable>& parameters = a->parameters();
        ASSERT_EQ(parameters[0].name(), "b");
        ASSERT_EQ(parameters[1].name(), "c");

        std::shared_ptr<const PddlType> d = task->type("d");
        ASSERT_EQ(parameters[0].type(), d);
        ASSERT_EQ(parameters[1].type(), d);
    }

    /**!
     * Tests parsing a predicates block with a single predicate with multiple parameters of different types
     */
    TEST(PddlFunction, single_predicate_multiple_type_parameters)
    {
        FileReader reader(
            "data/task_planning/pddl_parser/functions/predicates/single_predicate_multiple_type_parameters.pddl");
        mocks::MockPddlParser parser;
        parser.createPddlTask();
        parser.pddlTask()->setRequirement("typing");
        std::shared_ptr<PddlTask> task = parser.pddlTask();
        task->addType("d", "object");
        task->addType("e", "object");

        reader.checkNext(PddlSymbol::e_open_paren);
        parser.parseSingleDomainBlock(reader);
        ASSERT_EQ(task->numberOfPredicates(), 1);
        ASSERT_EQ(task->totalNumberOfFunctions(), 1);

        std::shared_ptr<const PddlFunction> a = task->function("a");
        ASSERT_EQ(a->parameters().size(), 2);

        const std::vector<PddlVariable>& parameters = a->parameters();
        ASSERT_EQ(parameters[0].name(), "b");
        ASSERT_EQ(parameters[1].name(), "c");

        std::shared_ptr<const PddlType> d = task->type("d");
        ASSERT_EQ(parameters[0].type(), d);
        std::shared_ptr<const PddlType> e = task->type("e");
        ASSERT_EQ(parameters[1].type(), e);
    }

    /**!
     * Tests parsing a predicates block with a multiple predicate with parameters
     */
    TEST(PddlFunction, multiple_predicates_multiple_type_parameters)
    {
        FileReader reader(
            "data/task_planning/pddl_parser/functions/predicates/multiple_predicates_multiple_parameters.pddl");
        mocks::MockPddlParser parser;
        parser.createPddlTask();
        parser.pddlTask()->setRequirement("typing");
        std::shared_ptr<PddlTask> task = parser.pddlTask();
        task->addType("d", "object");
        task->addType("e", "object");

        reader.checkNext(PddlSymbol::e_open_paren);
        parser.parseSingleDomainBlock(reader);
        ASSERT_EQ(task->numberOfPredicates(), 2);
        ASSERT_EQ(task->totalNumberOfFunctions(), 2);

        std::shared_ptr<const PddlFunction> a = task->function("a");
        ASSERT_EQ(a->parameters().size(), 2);

        const std::vector<PddlVariable>& parameters = a->parameters();
        ASSERT_EQ(parameters[0].name(), "b");
        ASSERT_EQ(parameters[1].name(), "c");

        std::shared_ptr<const PddlType> d = task->type("d");
        ASSERT_EQ(parameters[0].type(), d);
        std::shared_ptr<const PddlType> e = task->type("e");
        ASSERT_EQ(parameters[1].type(), e);

        std::shared_ptr<const PddlFunction> f = task->function("f");
        ASSERT_EQ(f->parameters().size(), 0);
    }

    /**!
     * Tests parsing a functions block with no functions
     */
    TEST(PddlFunction, empty_numeric_functions)
    {
        FileReader reader("data/task_planning/pddl_parser/functions/numeric/empty.pddl");
        mocks::MockPddlParser parser;
        parser.createPddlTask();
        parser.pddlTask()->setRequirement("typing");
        reader.checkNext(PddlSymbol::e_open_paren);
        parser.parseSingleDomainBlock(reader);
        std::shared_ptr<PddlTask> task = parser.pddlTask();
        ASSERT_EQ(task->totalNumberOfFunctions(), 0);
    }

    /**!
     * Tests parsing a functions block with a single numeric function with no parmeters
     */
    TEST(PddlFunction, single_function_no_parameters)
    {
        FileReader reader("data/task_planning/pddl_parser/functions/numeric/single_function_no_parameters.pddl");
        mocks::MockPddlParser parser;
        parser.createPddlTask();
        parser.pddlTask()->setRequirement("typing");
        reader.checkNext(PddlSymbol::e_open_paren);
        parser.parseSingleDomainBlock(reader);
        std::shared_ptr<PddlTask> task = parser.pddlTask();
        ASSERT_EQ(task->numberOfNumericFunctions(), 1);
        ASSERT_EQ(task->totalNumberOfFunctions(), 1);

        std::shared_ptr<const PddlFunction> a = task->function("a");
        ASSERT_EQ(a->parameters().size(), 0);
    }

    /**!
     * Tests parsing a functions block with a single function with multiple parameters of a single type
     */
    TEST(PddlFunction, single_function_single_type_parameters)
    {
        FileReader reader(
            "data/task_planning/pddl_parser/functions/numeric/single_function_single_type_parameters.pddl");
        mocks::MockPddlParser parser;
        parser.createPddlTask();
        parser.pddlTask()->setRequirement("typing");
        std::shared_ptr<PddlTask> task = parser.pddlTask();
        task->addType("d", "object");

        reader.checkNext(PddlSymbol::e_open_paren);
        parser.parseSingleDomainBlock(reader);
        ASSERT_EQ(task->numberOfNumericFunctions(), 1);
        ASSERT_EQ(task->totalNumberOfFunctions(), 1);

        std::shared_ptr<const PddlFunction> a = task->function("a");
        ASSERT_EQ(a->parameters().size(), 2);

        const std::vector<PddlVariable>& parameters = a->parameters();
        ASSERT_EQ(parameters[0].name(), "b");
        ASSERT_EQ(parameters[1].name(), "c");

        std::shared_ptr<const PddlType> d = task->type("d");
        ASSERT_EQ(parameters[0].type(), d);
        ASSERT_EQ(parameters[1].type(), d);
    }

    /**!
     * Tests parsing a functions block with a single function with multiple parameters of different types
     */
    TEST(PddlFunction, single_function_multiple_type_parameters)
    {
        FileReader reader(
            "data/task_planning/pddl_parser/functions/numeric/single_function_multiple_type_parameters.pddl");
        mocks::MockPddlParser parser;
        parser.createPddlTask();
        parser.pddlTask()->setRequirement("typing");
        std::shared_ptr<PddlTask> task = parser.pddlTask();
        task->addType("d", "object");
        task->addType("e", "object");

        reader.checkNext(PddlSymbol::e_open_paren);
        parser.parseSingleDomainBlock(reader);
        ASSERT_EQ(task->numberOfNumericFunctions(), 1);
        ASSERT_EQ(task->totalNumberOfFunctions(), 1);

        std::shared_ptr<const PddlFunction> a = task->function("a");
        ASSERT_EQ(a->parameters().size(), 2);

        const std::vector<PddlVariable>& parameters = a->parameters();
        ASSERT_EQ(parameters[0].name(), "b");
        ASSERT_EQ(parameters[1].name(), "c");

        std::shared_ptr<const PddlType> d = task->type("d");
        ASSERT_EQ(parameters[0].type(), d);
        std::shared_ptr<const PddlType> e = task->type("e");
        ASSERT_EQ(parameters[1].type(), e);
    }

    /**!
     * Tests parsing a functions block with a multiple functions with parameters
     */
    TEST(PddlFunction, multiple_functions_multiple_type_parameters)
    {
        FileReader reader(
            "data/task_planning/pddl_parser/functions/numeric/multiple_functions_multiple_parameters.pddl");
        mocks::MockPddlParser parser;
        parser.createPddlTask();
        parser.pddlTask()->setRequirement("typing");
        std::shared_ptr<PddlTask> task = parser.pddlTask();
        task->addType("d", "object");
        task->addType("e", "object");

        reader.checkNext(PddlSymbol::e_open_paren);
        parser.parseSingleDomainBlock(reader);
        ASSERT_EQ(task->numberOfNumericFunctions(), 2);
        ASSERT_EQ(task->totalNumberOfFunctions(), 2);

        std::shared_ptr<const PddlFunction> a = task->function("a");
        ASSERT_EQ(a->parameters().size(), 2);

        const std::vector<PddlVariable>& parameters = a->parameters();
        ASSERT_EQ(parameters[0].name(), "b");
        ASSERT_EQ(parameters[1].name(), "c");

        std::shared_ptr<const PddlType> d = task->type("d");
        ASSERT_EQ(parameters[0].type(), d);
        std::shared_ptr<const PddlType> e = task->type("e");
        ASSERT_EQ(parameters[1].type(), e);

        std::shared_ptr<const PddlFunction> f = task->function("f");
        ASSERT_EQ(f->parameters().size(), 0);
    }

    /**!
     * Tests parsing a functions block with a single function with an object return type
     */
    TEST(PddlFunction, object_function)
    {
        FileReader reader("data/task_planning/pddl_parser/functions/object/object_function.pddl");
        mocks::MockPddlParser parser;
        parser.createPddlTask();
        parser.pddlTask()->setRequirement("typing");
        std::shared_ptr<PddlTask> task = parser.pddlTask();
        task->addType("b", "object");

        reader.checkNext(PddlSymbol::e_open_paren);
        parser.parseSingleDomainBlock(reader);
        ASSERT_EQ(task->numberOfObjectFunctions(), 1);
        ASSERT_EQ(task->totalNumberOfFunctions(), 1);

        std::shared_ptr<const PddlFunction> a = task->function("a");
        ASSERT_EQ(a->parameters().size(), 0);
        std::shared_ptr<const PddlType> b = task->type("b");
        ASSERT_EQ(a->returnType(), b);
    }
}  // namespace grstapse::unittests