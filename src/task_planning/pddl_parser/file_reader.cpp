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
#include "grstapse/task_planning/pddl_parser/file_reader.hpp"

// Global
#include <fstream>

// External
#include <fmt/format.h>

// Local
#include "grstapse/common/utilities/error.hpp"
#include "grstapse/task_planning/pddl_parser/pddl_symbol.hpp"
#include "grstapse/task_planning/pddl_parser/pddl_token.hpp"

namespace grstapse
{
    FileReader::FileReader(const std::string& filename)
        : m_filename(filename)
        , m_position(0)
        , m_token_index(0)
    {
        std::ifstream is(filename);
        if(is.fail())
        {
            throw createRuntimeError(fmt::format("Could not find file: {0:s}", filename));
        }
        m_contents = std::string((std::istreambuf_iterator<char>(is)), std::istreambuf_iterator<char>());
        std::transform(m_contents.begin(), m_contents.end(), m_contents.begin(), ::tolower);

        m_tokens.reserve(1000);

        m_keywords = {{"define", PddlSymbol::e_define},
                      {"domain", PddlSymbol::e_domain},
                      {"problem", PddlSymbol::e_problem},
                      {"requirements", PddlSymbol::e_requirements},
                      {"types", PddlSymbol::e_types},
                      {"constants", PddlSymbol::e_constants},
                      {"predicates", PddlSymbol::e_predicates},
                      {"functions", PddlSymbol::e_functions},
                      {"durative-action", PddlSymbol::e_durative_action},
                      {"parameters", PddlSymbol::e_parameters},
                      {"duration", PddlSymbol::e_duration},
                      {"condition", PddlSymbol::e_condition},
                      {"effect", PddlSymbol::e_effect},
                      {"objects", PddlSymbol::e_objects},
                      {"init", PddlSymbol::e_init},
                      {"goal", PddlSymbol::e_goal},
                      {"at", PddlSymbol::e_at},
                      {"start", PddlSymbol::e_start},
                      {"end", PddlSymbol::e_end},
                      {"over", PddlSymbol::e_over},
                      {"all", PddlSymbol::e_all},
                      {"and", PddlSymbol::e_and},
                      {"or", PddlSymbol::e_or},
                      {"not", PddlSymbol::e_not},
                      {"forall", PddlSymbol::e_forall},
                      {"when", PddlSymbol::e_when},
                      {"metric", PddlSymbol::e_metric},
                      {"maximize", PddlSymbol::e_maximize},
                      {"minimize", PddlSymbol::e_minimize},
                      {"total-time", PddlSymbol::e_total_time}};
    }

    std::shared_ptr<PddlToken> FileReader::next()
    {
        if(m_token_index < 0)
        {
            return m_tokens[m_tokens.size() + m_token_index++];
        }

        // Skip white space
        skipWhiteSpace();

        // Skip comments
        while(m_position < m_contents.size())
        {
            if(m_contents[m_position] == ';')
            {
                while(m_position < m_contents.size() && m_contents[m_position] != '\n')
                {
                    ++m_position;
                }
                skipWhiteSpace();
            }
            else
            {
                break;
            }
        }

        std::shared_ptr<PddlToken> token = nullptr;
        if(m_position < m_contents.size())
        {
            token = getToken();
        }

        if(token == nullptr)
        {
            throw createRuntimeError("Reached the end of file while trying to get next token.");
        }
        m_tokens.push_back(token);
        return token;
    }

    void FileReader::skipWhiteSpace()
    {
        while(m_position < m_contents.size() && m_contents[m_position] <= ' ')
        {
            if(m_contents[m_position] == '\n')
            {
                ++m_linenumber;
            }
            ++m_position;
        }
    }

    bool is_letter(char c)
    {
        return c >= 'a' && c <= 'z';
    }

    bool is_number(char c)
    {
        return c >= '0' && c <= '9';
    }

    std::shared_ptr<PddlToken> FileReader::getToken()
    {
        std::shared_ptr<PddlToken> token = nullptr;
        switch(m_contents[m_position])
        {
            case '(':
                token = std::make_shared<PddlToken>(PddlSymbol::e_open_paren);
                break;
            case ')':
                token = std::make_shared<PddlToken>(PddlSymbol::e_closed_paren);
                break;
            case ':':
                token = std::make_shared<PddlToken>(PddlSymbol::e_colon);
                break;
            case '=':
                token = std::make_shared<PddlToken>(PddlSymbol::e_equal);
                break;
            case '-':
                token = std::make_shared<PddlToken>(PddlSymbol::e_minus);
                break;
        }

        if(token != nullptr)
        {
            ++m_position;
            return token;
        }

        float value;
        if(getNumber(value))
        {
            return std::make_shared<PddlToken>(value);
        }

        unsigned int start = m_position++;
        while(m_position < m_contents.size() &&
              (is_letter(m_contents[m_position]) || is_number(m_contents[m_position]) ||
               m_contents[m_position] == '-' || m_contents[m_position] == '_'))
        {
            ++m_position;
        }
        std::string description = m_contents.substr(start, m_position - start);
        if(description[0] == '?')
        {
            return std::make_shared<PddlToken>(PddlSymbol::e_variable, description.substr(1));
        }
        else if(auto iter = m_keywords.find(description); iter != m_keywords.end())
        {
            return std::make_shared<PddlToken>(iter->second, description);
        }
        else
        {
            return std::make_shared<PddlToken>(PddlSymbol::e_name, description);
        }
    }

    bool FileReader::getNumber(float& value)
    {
        if(m_position >= m_contents.size() || !is_number(m_contents[m_position]))
        {
            return false;
        }

        value = 0;
        while(m_position < m_contents.size() && is_number(m_contents[m_position]))
        {
            value = value * 10 + static_cast<int>(m_contents[m_position]) - static_cast<int>('0');
            ++m_position;
        }

        if(m_position < m_contents.size() && m_contents[m_position] == '.')
        {
            float decimals = 0;
            float digits   = 1;
            ++m_position;
            while(m_position < m_contents.size() && is_number(m_contents[m_position]))
            {
                decimals = decimals * 10 + static_cast<int>(m_contents[m_position]) - static_cast<int>('0');
                digits *= 10;
                ++m_position;
            }
            value += decimals / digits;
        }
        return true;
    }

    std::shared_ptr<PddlToken> FileReader::checkNext(PddlSymbol symbol)
    {
        auto token = next();
        if(token != nullptr && token->symbol() == symbol)
        {
            return token;
        }
        throw createLogicError(fmt::format("Error in file {0} (line {1}): Unexpected token '{2}'",
                                           m_filename,
                                           m_linenumber,
                                           magic_enum::enum_name(token->symbol())));
    }

    std::shared_ptr<PddlToken> FileReader::checkNext(const std::vector<PddlSymbol>& symbols)
    {
        auto token = next();
        for(PddlSymbol symbol: symbols)
        {
            if(token->symbol() == symbol)
            {
                return token;
            }
        }
        throw createLogicError(fmt::format("Error in file {} (line {}): Unexpected token '{}'",
                                           m_filename,
                                           m_linenumber,
                                           magic_enum::enum_name(token->symbol())));
    }

    void FileReader::undo()
    {
        --m_token_index;
    }

    std::string FileReader::readName()
    {
        auto token = next();
        if(token->symbol() != PddlSymbol::e_name && token->symbol() != PddlSymbol::e_at &&
           token->symbol() != PddlSymbol::e_over && token->symbol() != PddlSymbol::e_objects)
        {
            throw createLogicError(fmt::format("Error in file {} (line {}): Name expected, but token '{}' found",
                                               m_filename,
                                               m_linenumber,
                                               magic_enum::enum_name(token->symbol())));
        }
        return token->description();
    }
}  // namespace grstapse