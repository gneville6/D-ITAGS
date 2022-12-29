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

namespace grstapse::unittests
{
    TEST(PddlRequirements, set_adl)
    {
        PddlRequirements requirements;
        requirements.set("adl");
        ASSERT_TRUE(requirements.strips);
        ASSERT_TRUE(requirements.typing);
        ASSERT_TRUE(requirements.disjunctive_preconditions);
        ASSERT_TRUE(requirements.equality);
        ASSERT_TRUE(requirements.existential_preconditions);
        ASSERT_TRUE(requirements.universal_preconditions);
        ASSERT_TRUE(requirements.conditional_effects);
    }

    TEST(PddlRequirements, set_conditional_effects)
    {
        PddlRequirements requirements;
        requirements.set("conditional-effects");
        ASSERT_TRUE(requirements.conditional_effects);
    }

    TEST(PddlRequirements, set_constraints)
    {
        PddlRequirements requirements;
        EXPECT_ANY_THROW(requirements.set("constraints"));
    }

    TEST(PddlRequirements, set_derived_predicates)
    {
        PddlRequirements requirements;
        EXPECT_ANY_THROW(requirements.set("derived-predicates"));
    }

    TEST(PddlRequirements, set_disjunctive_preconditions)
    {
        PddlRequirements requirements;
        requirements.set("disjunctive-preconditions");
        ASSERT_TRUE(requirements.disjunctive_preconditions);
    }

    TEST(PddlRequirements, set_durative_actions)
    {
        PddlRequirements requirements;
        requirements.set("durative-actions");
        ASSERT_TRUE(requirements.durative_actions);
    }

    TEST(PddlRequirements, set_durative_inequalities)
    {
        PddlRequirements requirements;
        EXPECT_ANY_THROW(requirements.set("durative-inequalities"));
    }

    TEST(PddlRequirements, set_equality)
    {
        PddlRequirements requirements;
        requirements.set("equality");
        ASSERT_TRUE(requirements.equality);
    }

    TEST(PddlRequirements, set_existential_preconditions)
    {
        PddlRequirements requirements;
        requirements.set("existential-preconditions");
        ASSERT_TRUE(requirements.existential_preconditions);
    }

    TEST(PddlRequirements, set_negative_preconditions)
    {
        PddlRequirements requirements;
        requirements.set("negative-preconditions");
        ASSERT_TRUE(requirements.negative_preconditions);
    }

    TEST(PddlRequirements, set_numeric_fluents)
    {
        PddlRequirements requirements;
        EXPECT_ANY_THROW(requirements.set("numeric-fluents"));
    }

    TEST(PddlRequirements, set_preferences)
    {
        PddlRequirements requirements;
        EXPECT_ANY_THROW(requirements.set("preferences"));
    }

    TEST(PddlRequirements, set_quantified_preconditions)
    {
        PddlRequirements requirements;
        requirements.set("quantified-preconditions");
        ASSERT_TRUE(requirements.existential_preconditions);
        ASSERT_TRUE(requirements.universal_preconditions);
    }

    TEST(PddlRequirements, set_strips)
    {
        PddlRequirements requirements;
        requirements.set("strips");
        ASSERT_TRUE(requirements.strips);
    }

    TEST(PddlRequirements, set_timed_initial_literals)
    {
        PddlRequirements requirements;
        requirements.set("timed-initial-literals");
        ASSERT_TRUE(requirements.timed_initial_literals);
    }

    TEST(PddlRequirements, set_typing)
    {
        PddlRequirements requirements;
        requirements.set("typing");
        ASSERT_TRUE(requirements.typing);
    }

    TEST(PddlRequirements, set_universal_preconditions)
    {
        PddlRequirements requirements;
        requirements.set("universal-preconditions");
        ASSERT_TRUE(requirements.universal_preconditions);
    }

    TEST(PddlRequirements, read_requirements_block)
    {
        FileReader reader("data/task_planning/pddl_parser/requirements.pddl");

        mocks::MockPddlParser parser;
        parser.createPddlTask();
        reader.checkNext(PddlSymbol::e_open_paren);
        parser.parseSingleDomainBlock(reader);

        std::shared_ptr<const PddlRequirements> requirements = parser.pddlTask()->requirements();
        ASSERT_TRUE(requirements->conditional_effects);
        ASSERT_TRUE(requirements->disjunctive_preconditions);
        ASSERT_TRUE(requirements->durative_actions);
        ASSERT_TRUE(requirements->equality);
        ASSERT_TRUE(requirements->existential_preconditions);
        ASSERT_TRUE(requirements->negative_preconditions);
        ASSERT_TRUE(requirements->strips);
        ASSERT_TRUE(requirements->timed_initial_literals);
        ASSERT_TRUE(requirements->typing);
        ASSERT_TRUE(requirements->universal_preconditions);
    }
}  // namespace grstapse::unittests