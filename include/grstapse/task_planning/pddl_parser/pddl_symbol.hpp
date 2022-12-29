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

// External
//// Magic Enum
#include <magic_enum/magic_enum.hpp>

namespace grstapse
{
    enum class PddlSymbol
    {
        //
        e_name,
        e_variable,
        e_number,
        // Characters
        e_open_paren,
        e_closed_paren,
        e_colon,
        // keywords
        e_define,
        e_domain,
        e_problem,
        // block types
        e_requirements,
        e_types,
        e_constants,
        e_predicates,
        e_functions,
        e_durative_action,
        e_parameters,
        e_duration,
        e_condition,
        e_effect,
        e_objects,
        e_init,
        e_goal,
        // Condition/Effect
        e_at,
        e_start,
        e_end,
        e_over,
        e_all,
        e_and,
        e_or,
        e_not,
        e_equal,
        e_forall,
        e_when,
        // Operator
        e_minus,
        // Metric
        e_metric,
        e_maximize,
        e_minimize,
        e_total_time
    };
}  // namespace grstapse