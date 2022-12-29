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
#include "grstapse/task_planning/pddl_parser/pddl_parser.hpp"

// Local
#include "grstapse/common/utilities/error.hpp"
#include "grstapse/task_planning/pddl_parser/file_reader.hpp"
#include "grstapse/task_planning/pddl_parser/pddl_duration.hpp"
#include "grstapse/task_planning/pddl_parser/pddl_requirements.hpp"
#include "grstapse/task_planning/pddl_parser/pddl_symbol.hpp"
#include "grstapse/task_planning/pddl_parser/pddl_task.hpp"
#include "grstapse/task_planning/pddl_parser/pddl_token.hpp"
#include "grstapse/task_planning/pddl_parser/pddl_variable.hpp"

namespace grstapse
{
    std::vector<PddlSymbol> PddlParser::s_domain_block_symbols = {PddlSymbol::e_requirements,
                                                                  PddlSymbol::e_types,
                                                                  PddlSymbol::e_constants,
                                                                  PddlSymbol::e_predicates,
                                                                  PddlSymbol::e_functions,
                                                                  PddlSymbol::e_durative_action};

    PddlParser::PddlParser()
        : m_task(nullptr)
    {}

    void PddlParser::parseDomain(const std::string& filename)
    {
        m_task = std::make_shared<PddlTask>();

        FileReader reader(filename);
        std::string domain_name = parseHeader(reader, PddlSymbol::e_domain);
        m_task->setDomainName(domain_name);

        parseDomainBlocks(reader);
    }

    void PddlParser::parseProblem(const std::string& filename)
    {
        assert(m_task);
    }

    std::shared_ptr<const PddlTask> PddlParser::pddlTask()
    {
        return m_task;
    }

    void PddlParser::parseDomainBlocks(FileReader& reader)
    {
        const std::vector<PddlSymbol> parentheses = {PddlSymbol::e_open_paren, PddlSymbol::e_closed_paren};
        auto token                                = reader.checkNext(parentheses);
        while(token->symbol() == PddlSymbol::e_open_paren)
        {
            parseSingleDomainBlock(reader);
            token = reader.checkNext(parentheses);
        }
    }

    void PddlParser::parseSingleDomainBlock(FileReader& reader)
    {
        reader.checkNext(PddlSymbol::e_colon);
        auto token = reader.checkNext(s_domain_block_symbols);
        switch(token->symbol())
        {
            case PddlSymbol::e_requirements:
                parseRequirements(reader);
                break;
            case PddlSymbol::e_types:
                parseTypes(reader);
                break;
            case PddlSymbol::e_constants:
                parseConstants(reader);
                break;
            case PddlSymbol::e_predicates:
                parsePredicates(reader);
                break;
            case PddlSymbol::e_functions:
                parseFunctions(reader);
                break;
            case PddlSymbol::e_durative_action:
                parseDurativeAction(reader);
                break;
        }
    }

    void PddlParser::parseRequirements(FileReader& reader)
    {
        auto token = reader.checkNext({PddlSymbol::e_colon, PddlSymbol::e_closed_paren});
        while(token->symbol() == PddlSymbol::e_colon)
        {
            m_task->setRequirement(reader.readName());
            token = reader.checkNext({PddlSymbol::e_colon, PddlSymbol::e_closed_paren});
        }
    }

    void PddlParser::parseTypes(FileReader& reader)
    {
        if(!m_task->requirements()->typing)
        {
            throw createLogicError("The typing requirement must be set to have the types block");
        }

        auto token = reader.checkNext({PddlSymbol::e_closed_paren, PddlSymbol::e_name});
        std::pair<std::vector<std::string>, std::string> typed_list;
        while(token->symbol() != PddlSymbol::e_closed_paren)
        {
            reader.undo();
            auto [list, list_type] = parseTypedList(reader);
            for(const std::string& s: list)
            {
                m_task->addType(s, list_type);
            }

            token = reader.checkNext({PddlSymbol::e_closed_paren, PddlSymbol::e_name});
        }
    }

    void PddlParser::parsePredicates(FileReader& reader)
    {
        auto token = reader.checkNext({PddlSymbol::e_closed_paren, PddlSymbol::e_open_paren});
        while(token->symbol() == PddlSymbol::e_open_paren)
        {
            // Read the name of the predicate
            std::string name = reader.readName();

            // Read all the typed parameters
            std::vector<PddlVariable> parameters = parseVariableList(reader);

            m_task->addPredicate(name, parameters);
            token = reader.checkNext({PddlSymbol::e_closed_paren, PddlSymbol::e_open_paren});
        }
    }

    void PddlParser::parseFunctions(FileReader& reader)
    {
        auto token = reader.checkNext({PddlSymbol::e_closed_paren, PddlSymbol::e_open_paren});
        while(token->symbol() == PddlSymbol::e_open_paren)
        {
            // Read the name of the function
            std::string name = reader.readName();

            // Read all the typed parameters
            std::vector<PddlVariable> parameters = parseVariableList(reader);
            token = reader.checkNext({PddlSymbol::e_closed_paren, PddlSymbol::e_open_paren, PddlSymbol::e_minus});

            // Check if there is a return type for the function (making it an object function);
            if(token->symbol() == PddlSymbol::e_minus)
            {
                std::string type_name = reader.readName();

                // Is actually a numeric function
                if(type_name == "number")
                {
                    m_task->addFunction(name, parameters);
                }
                // Is an object function
                else
                {
                    std::shared_ptr<const PddlType> type = m_task->type(type_name);
                    m_task->addFunction(name, parameters, type);
                }
                token = reader.checkNext({PddlSymbol::e_closed_paren, PddlSymbol::e_open_paren});
            }
            // Is a numeric function
            else
            {
                m_task->addFunction(name, parameters);
            }
        }
    }

    void PddlParser::parseConstants(FileReader& reader)
    {
        auto token = reader.checkNext({PddlSymbol::e_closed_paren, PddlSymbol::e_name});
        std::pair<std::vector<std::string>, std::string> typed_list;
        while(token->symbol() != PddlSymbol::e_closed_paren)
        {
            reader.undo();
            auto [list, list_type] = parseTypedList(reader);
            for(const std::string& s: list)
            {
                m_task->addConstant(s, list_type);
            }

            token = reader.checkNext({PddlSymbol::e_closed_paren, PddlSymbol::e_name});
        }
    }

    void PddlParser::parseDurativeAction(FileReader& reader)
    {
        // Name
        std::string name = reader.readName();

        // Parameters
        reader.checkNext(PddlSymbol::e_colon);
        reader.checkNext(PddlSymbol::e_parameters);
        reader.checkNext(PddlSymbol::e_open_paren);
        std::vector<PddlVariable> parameters = parseVariableList(reader);

        // Duration
        reader.checkNext(PddlSymbol::e_colon);
        reader.checkNext(PddlSymbol::e_duration);
        std::shared_ptr<const PddlDuration> duration = parseDuration(reader, parameters);

        reader.checkNext(PddlSymbol::e_colon);
        std::shared_ptr<PddlToken> token = reader.checkNext({PddlSymbol::e_condition, PddlSymbol::e_effect});

        // Condition
        std::shared_ptr<const PddlCondition> condition;
        if(token->symbol() == PddlSymbol::e_condition)
        {
            condition = parseCondition(reader, parameters);
            reader.checkNext(PddlSymbol::e_colon);
        }
        else
        {
            // todo: empty condition
            reader.undo();
        }

        // Effect
        reader.checkNext(PddlSymbol::e_effect);
        std::shared_ptr<const PddlEffect> effect = parseEffect(reader, parameters);

        // m_task->addAction(name, parameters, duration, condition, effect);
        reader.checkNext(PddlSymbol::e_colon);
    }

    void PddlParser::parseObjects(FileReader& reader)
    {
        auto token = reader.checkNext({PddlSymbol::e_closed_paren, PddlSymbol::e_name});
        std::pair<std::vector<std::string>, std::string> typed_list;
        while(token->symbol() != PddlSymbol::e_closed_paren)
        {
            reader.undo();
            auto [list, list_type] = parseTypedList(reader);
            for(const std::string& s: list)
            {
                m_task->addObject(s, list_type);
            }

            token = reader.checkNext({PddlSymbol::e_closed_paren, PddlSymbol::e_name});
        }
    }

    void PddlParser::parseInit() {}

    void PddlParser::parseGoal() {}

    void PddlParser::parseMetric() {}

    std::string PddlParser::parseHeader(FileReader& reader, PddlSymbol symbol)
    {
        reader.checkNext(PddlSymbol::e_open_paren);
        reader.checkNext(PddlSymbol::e_define);
        reader.checkNext(PddlSymbol::e_open_paren);
        reader.checkNext(symbol);
        std::string name = reader.readName();
        reader.checkNext(PddlSymbol::e_closed_paren);
        return name;
    }

    std::pair<std::vector<std::string>, std::string> PddlParser::parseTypedList(FileReader& reader)
    {
        auto token = reader.checkNext(
            {PddlSymbol::e_name, PddlSymbol::e_variable, PddlSymbol::e_minus, PddlSymbol::e_closed_paren});
        std::vector<std::string> list;
        while(token->symbol() == PddlSymbol::e_name || token->symbol() == PddlSymbol::e_variable)
        {
            list.push_back(token->description());
            token = reader.checkNext(
                {PddlSymbol::e_name, PddlSymbol::e_variable, PddlSymbol::e_minus, PddlSymbol::e_closed_paren});
        }
        if(token->symbol() == PddlSymbol::e_minus)
        {
            return std::pair(list, reader.readName());
        }
        // else token->symbol() == PddlSymbol::e_closed_paren
        reader.undo();
        return std::pair(list, "#object");
    }

    std::vector<PddlVariable> PddlParser::parseVariableList(FileReader& reader)
    {
        // Read all the typed parameters
        std::vector<PddlVariable> parameters;

        std::shared_ptr<PddlToken> token = reader.checkNext({PddlSymbol::e_closed_paren, PddlSymbol::e_variable});
        while(token->symbol() == PddlSymbol::e_variable)
        {
            reader.undo();

            // Read all the parameters of one type
            auto [list, list_type]               = parseTypedList(reader);
            std::shared_ptr<const PddlType> type = m_task->type(list_type);
            for(const std::string& s: list)
            {
                parameters.emplace_back(s, type);
            }
            token = reader.checkNext({PddlSymbol::e_closed_paren, PddlSymbol::e_variable});
        }
        return parameters;
    }

    std::shared_ptr<const PddlDuration> PddlParser::parseDuration(FileReader& reader,
                                                                  const std::vector<PddlVariable>& parameters)
    {
        reader.checkNext(PddlSymbol::e_open_paren);
        std::shared_ptr<const PddlToken> token =
            reader.checkNext({PddlSymbol::e_and, PddlSymbol::e_closed_paren, PddlSymbol::e_equal, PddlSymbol::e_at});
        if(token->symbol() == PddlSymbol::e_closed_paren)
        {
            throw createLogicError("We currently do not handle this type of duration equation");
        }
        else if(token->symbol() == PddlSymbol::e_at)
        {
            throw createLogicError("We currently do not handle this type of duration equation");
        }
        else if(token->symbol() == PddlSymbol::e_and)
        {
            if(!m_task->requirements()->durative_inequalities)
            {
                throw createLogicError("Cannot have durative inequalities without the requirement set.");
            }
            throw createLogicError("We currently do not handle this type of duration equation");
        }
        else  // =
        {
            std::shared_ptr<const PddlToken> aux = reader.checkNext(PddlSymbol::e_variable);
            if(aux->description() != "duration")
            {
                throw createLogicError("Variable ?duration expected");
            }
            float exp = reader.checkNext(PddlSymbol::e_number)->value();

            // TODO: Maybe add numeric expressions?
            // std::shared_ptr<const PddlNumericExpression> exp = parseNumericExpression(parameters);

            PddlComparator comparator;
            switch(token->symbol())
            {
                case PddlSymbol::e_equal:
                {
                    comparator = PddlComparator::e_eq;
                    break;
                }
                default:
                {
                    throw createLogicError("We currently do not handle this type of duration equation");
                }
            }

            auto duration = std::make_shared<PddlDuration>(comparator, exp);
            reader.checkNext(PddlSymbol::e_closed_paren);
            return duration;
        }
    }
    std::shared_ptr<const PddlCondition> PddlParser::parseCondition(FileReader& reader,
                                                                    const std::vector<PddlVariable>& parameters)
    {
        return std::shared_ptr<const PddlCondition>();
    }
    std::shared_ptr<const PddlEffect> PddlParser::parseEffect(FileReader& reader,
                                                              const std::vector<PddlVariable>& parameters)
    {
        return std::shared_ptr<const PddlEffect>();
    }
}  // namespace grstapse