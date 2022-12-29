/*
 * Graphically Recursive Simultaneous Task Allocation, Planning,
 * Scheduling, and Execution
 *
 * Copyright (C) 2020-2022
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
#include <ostream>
// External
#include <nlohmann/json.hpp>

namespace grstapse
{
    /**!
     * Base class for statistics recorded during search
     *
     * \note This class must be derived from
     */
    class SearchStatisticsBase
    {
       public:
        //! \brief Serializes to json
        virtual void serializeToJson(nlohmann::json& j) const = 0;

        //! \brief Prints a human readable representation of the statistics to a stream
        virtual std::ostream& print(std::ostream& os) const = 0;

        //! \brief Prints a human readable representation of the statistics to the screen
        void printStatistics() const;

       protected:
        SearchStatisticsBase();
    };

    /**!
     * Concept to force a type to derive from SearchStatisticsBase
     *
     * \tparam T A derivative of SearchStatisticsBase
     */
    template <typename T>
    concept SearchStatisticsDeriv = std::derived_from<T, SearchStatisticsBase>;
}  // namespace grstapse