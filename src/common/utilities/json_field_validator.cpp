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
#include "grstapse/common/utilities/json_field_validator.hpp"

// External
#include <fmt/format.h>
#include <magic_enum/magic_enum.hpp>
// Local
#include "grstapse/common/utilities/error.hpp"

namespace grstapse
{
    void validate(const nlohmann::json& j, const std::vector<std::pair<std::string, nlohmann::json::value_t>>& fields)
    {
        for(const std::pair<std::string, nlohmann::json::value_t>& p: fields)
        {
            const std::string& field_name = p.first;
            if(!j.contains(field_name))
            {
                throw createLogicError(fmt::format("json is missing field '{0:s}'", field_name));
            }

            const nlohmann::json& f = j.at(field_name);
            if(f.type() != p.second)
            {
                throw createLogicError(
                    fmt::format("json field '{0:s}' should be of type '{1:s}' however is instead of type '{2:s}'",
                                field_name,
                                magic_enum::enum_name(p.second),
                                magic_enum::enum_name(f.type())));
            }
        }
    }
}  // namespace grstapse