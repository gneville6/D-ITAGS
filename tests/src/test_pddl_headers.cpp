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
#include <grstapse/task_planning/pddl_parser/pddl_symbol.hpp>

namespace grstapse::unittests
{
    TEST(PddlHeaders, domain)
    {
        FileReader reader("data/task_planning/pddl_parser/domain_header.pddl");
        mocks::MockPddlParser parser;
        ASSERT_STREQ(parser.parseHeader(reader, PddlSymbol::e_domain).c_str(), "test");
    }

    TEST(PddlHeaders, problem)
    {
        FileReader reader("data/task_planning/pddl_parser/problem_header.pddl");
        mocks::MockPddlParser parser;
        ASSERT_STREQ(parser.parseHeader(reader, PddlSymbol::e_problem).c_str(), "test_problem");
    }
}  // namespace grstapse::unittests