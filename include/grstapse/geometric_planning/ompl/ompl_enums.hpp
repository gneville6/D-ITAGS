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
#include <memory>
// External
#include <nlohmann/json.hpp>

namespace grstapse
{
    enum class OmplStateSpaceType : uint8_t
    {
        e_unknown = 0,
        e_se2,
        e_se3,
        e_so3
    };
    NLOHMANN_JSON_SERIALIZE_ENUM(OmplStateSpaceType,
                                 {{OmplStateSpaceType::e_unknown, "unknown"},
                                  {OmplStateSpaceType::e_se2, "se2"},
                                  {OmplStateSpaceType::e_se3, "se3"},
                                  {OmplStateSpaceType::e_so3, "so3"}});

    enum class OmplGoalType : uint8_t
    {
        e_unknown = 0,
        e_state,
        e_set_of_states,
        e_space
    };
    NLOHMANN_JSON_SERIALIZE_ENUM(OmplGoalType,
                                 {{OmplGoalType::e_unknown, "unknown"},
                                  {OmplGoalType::e_state, "state"},
                                  {OmplGoalType::e_set_of_states, "set_of_states"},
                                  {OmplGoalType::e_space, "space"}});
}  // namespace grstapse