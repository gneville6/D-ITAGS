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
#include <grstapse/task_planning/pddl_parser/pddl_duration.hpp>
#include <grstapse/task_planning/pddl_parser/pddl_enums.hpp>
#include <grstapse/task_planning/pddl_parser/pddl_function.hpp>
#include <grstapse/task_planning/pddl_parser/pddl_requirements.hpp>
#include <grstapse/task_planning/pddl_parser/pddl_symbol.hpp>
#include <grstapse/task_planning/pddl_parser/pddl_type.hpp>
#include <grstapse/task_planning/pddl_parser/pddl_variable.hpp>

namespace grstapse::unittests
{
    /**!
     * We currently do not handle other types of duration constraints and so throw an exception
     */
    TEST(PddlDuration, empty)
    {
        FileReader reader("data/task_planning/pddl_parser/durative_actions/duration/empty.pddl");
        mocks::MockPddlParser parser;
        parser.createPddlTask();
        reader.checkNext(PddlSymbol::e_colon);
        reader.checkNext(PddlSymbol::e_duration);
        ASSERT_ANY_THROW(parser.parseDuration(reader, std::vector<PddlVariable>()));
        // ASSERT_EQ(duration->comparator(), PddlComparator::e_eq);
        // ASSERT_EQ(duration->value(), 0.0);
    }

    /**!
     * Tests a fixed duration constraint
     */
    TEST(PddlDuration, equal)
    {
        FileReader reader("data/task_planning/pddl_parser/durative_actions/duration/equal.pddl");
        mocks::MockPddlParser parser;
        parser.createPddlTask();
        reader.checkNext(PddlSymbol::e_colon);
        reader.checkNext(PddlSymbol::e_duration);
        std::shared_ptr<const PddlDuration> duration = parser.parseDuration(reader, std::vector<PddlVariable>());
        ASSERT_EQ(duration->comparator(), PddlComparator::e_eq);
        ASSERT_EQ(duration->value(), 2.0);
    }

    /**!
     * We currently do not handle this type of duration constraints and so throw an exception
     */
    TEST(PddlDuration, inequality)
    {
        FileReader reader("data/task_planning/pddl_parser/durative_actions/duration/inequality.pddl");
        mocks::MockPddlParser parser;
        parser.createPddlTask();
        reader.checkNext(PddlSymbol::e_colon);
        reader.checkNext(PddlSymbol::e_duration);
        EXPECT_ANY_THROW(parser.parseDuration(reader, std::vector<PddlVariable>()));
    }
}  // namespace grstapse::unittests