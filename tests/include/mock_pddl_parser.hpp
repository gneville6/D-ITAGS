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

// Project
#include <grstapse/task_planning/pddl_parser/pddl_parser.hpp>

namespace grstapse::mocks
{
    /**
     * Mock which reveals some of the protect functions in pddl parser as public for testing
     */
    class MockPddlParser : public PddlParser
    {
       public:
        //! Creates a pddl task without needing the parseDomain function
        void createPddlTask()
        {
            m_task = std::make_shared<PddlTask>();
        }

        //! \returns A mutable PddlTask
        std::shared_ptr<PddlTask> pddlTask()
        {
            return m_task;
        }

        using PddlParser::parseDomainBlocks;
        using PddlParser::parseDuration;
        using PddlParser::parseHeader;
        using PddlParser::parseRequirements;
        using PddlParser::parseSingleDomainBlock;
    };
}  // namespace grstapse::mocks