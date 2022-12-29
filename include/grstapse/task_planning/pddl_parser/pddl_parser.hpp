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
#include <memory>
#include <string>
#include <vector>

// Local
#include "grstapse/task_planning/pddl_parser/pddl_task.hpp"

namespace grstapse
{
    enum class PddlSymbol;
    class FileReader;
    class PddlTask;
    class PddlDuration;
    class PddlCondition;
    class PddlEffect;
    class PddlVariable;

    class PddlParser
    {
       public:
        //! \brief Default Constructor
        PddlParser();

        //! Parse a pddl domain file
        void parseDomain(const std::string& filename);

        //! Parse a pddl problem file
        void parseProblem(const std::string& filename);

        //! \returns The task parsed from pddl domain and problem files
        std::shared_ptr<const PddlTask> pddlTask();

       protected:
        //! Loop through and parse all the block in a domain file
        void parseDomainBlocks(FileReader& reader);

        //! Select the parser for a single block
        void parseSingleDomainBlock(FileReader& reader);

        /**!
         * Parse the requirements block
         *
         * <require-def> ::= (:requirements <require-key>+)
         */
        void parseRequirements(FileReader& reader);

        /**!
         * Parse the types block
         *
         * <types-def> ::= (:types <typed list (name)>)
         */
        void parseTypes(FileReader& reader);

        /**!
         * Pares the predicates block
         *
         * <predicates-def> ::= (:predicates <atomic formula skeleton>+)
         * <atomic formula skeleton> ::= (<predicate> <typed list (variable)>)
         * <predicate> ::= <name>
         * <variable> ::= ?<name>
         */
        void parsePredicates(FileReader& reader);

        /**!
         * Parse the functions block
         *
         * <functions-def> ::= (:functions <atomic function skeleton>+)
         * <atomic function skeleton> ::= (<function-symbol> <typed list (variable)>)
         * <atomic function skeleton> ::= (<function-symbol> <typed list (variable)>) - <function-type>
         * <function-symbol> ::= <name>
         * <variable> ::= ?<name>
         * <function type> ::= number
         * <function type> ::= <name>
         */
        void parseFunctions(FileReader& reader);

        /**!
         * Parse the constants block
         *
         * <constants-def> ::= (:constants <typed list (name)>)
         */
        void parseConstants(FileReader& reader);

        /**!
         *
         * <durative-action-def> ::= (:durative-action <da-symobol>
         *                              :parameters (<type list (variable)>) <da-def-body>)
         * <da-def-body> ::= :duration <duration-constraint>
         *                   :condition <emptyOr (da-GD)>
         *                   :effect <emptyOr (da-effect)>
         */
        void parseDurativeAction(FileReader& reader);

        /**!
         * Parse the constants block
         *
         * <objects-def> ::= (:objects <typed list (name)>)
         */
        void parseObjects(FileReader& reader);

        void parseInit();
        void parseGoal();
        void parseMetric();

        /**
         * Parse the header of a pddl file
         *
         * \returns The domain or problem name
         */
        std::string parseHeader(FileReader& reader, PddlSymbol symbol);

        /**
         * Parse a list followed by a type
         *
         * <typed list(x)> ::= x*
         *
         * if :typing requirement is set:
         * <typed list(x)> ::= x+ - <type> <typed list (x)>
         */
        std::pair<std::vector<std::string>, std::string> parseTypedList(FileReader& reader);

        /**!
         *
         */
        std::vector<PddlVariable> parseVariableList(FileReader& reader);

        /**!
         * if duration-inequalities: <duraiton-constraint> ::= (and <simple-duration-constraint>+)
         *
         * <duration-constraint> ::= ()
         * <duration-constraint> ::= <simple-duration-constraint>
         * <simple-duration-constraint> ::= (<d-op> ?duration <d-value>)
         * <simple-duration-constraint> ::= (at <time-specified> <simple-duration-constraint>)
         * if duration-inequalities: <d-op> ::= >=
         * if duration-inequalities: <d-op> ::= <=
         * <d-op> ::= =
         * <d-value> ::= <number>
         * if numeric-fluents: <d-value> ::= <f-exp>
         */
        std::shared_ptr<const PddlDuration> parseDuration(FileReader& reader,
                                                          const std::vector<PddlVariable>& parameters);
        std::shared_ptr<const PddlCondition> parseCondition(FileReader& reader,
                                                            const std::vector<PddlVariable>& parameters);
        std::shared_ptr<const PddlEffect> parseEffect(FileReader& reader, const std::vector<PddlVariable>& parameters);

        std::shared_ptr<PddlTask> m_task;

        static std::vector<PddlSymbol> s_domain_block_symbols;
    };
}  // namespace grstapse