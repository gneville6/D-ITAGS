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

// External
#include <robin_hood/robin_hood.hpp>

namespace grstapse
{
    class PddlToken;
    enum class PddlSymbol;

    class FileReader
    {
       public:
        /**
         *
         * \param filename
         */
        FileReader(const std::string& filename);

        //! \returns The next token from the file
        std::shared_ptr<PddlToken> next();

        //! \returns The next token if it is the specified \p symbol
        std::shared_ptr<PddlToken> checkNext(PddlSymbol symbol);

        //! \returns The next token if it is one of the specfied \p symbols
        std::shared_ptr<PddlToken> checkNext(const std::vector<PddlSymbol>& symbols);

        //! Reverts the reader back one token
        void undo();

        std::string readName();

       private:
        //! \brief Skips the white space in the file
        void skipWhiteSpace();

        //! Creates the next token from the file
        std::shared_ptr<PddlToken> getToken();

        /**!
         * \brief Gets the value of the next token if it is a number
         *
         * \p value Gets filled if the token is a number
         *
         * \returns Whether the token is a number
         */
        bool getNumber(float& value);

        std::string m_filename;

        std::string m_contents;
        unsigned int m_position;
        unsigned int m_linenumber;

        robin_hood::unordered_map<std::string, PddlSymbol> m_keywords;

        std::vector<std::shared_ptr<PddlToken>> m_tokens;
        int m_token_index;
    };
}  // namespace grstapse