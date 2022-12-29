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

// Local
#include "grstapse/task_planning/pddl_parser/pddl_base.hpp"

namespace grstapse
{
    //! \brief A container for what requirements are set for a pddl domain
    struct PddlRequirements : public PddlBase
    {
        //! \brief Creates the container with all requirements set to false
        PddlRequirements();

        //! \brief Set the requirement boolean based on \p requirement
        void set(const std::string& requirement);

        /**!
         * \brief Prints a PDDL representation to the given stream
         *
         * \param os The stream to print to
         * \returns \p os
         */
        std::ostream& print(std::ostream& os) const override;

        bool conditional_effects;  // Allows for the usage of "when" in expressing action effect
        bool constraints;          // Allows for the use of constraints (goals which much be satisfied in every state)
        bool continuous_effects;   // Allows the use of continuous effects on numeric fluents within durative actions
        bool derived_predicates;   // Allows the use of "derived" for derived predicates in the domain definition
        bool disjunctive_preconditions;  // Allows "or" in goals or preconditions
        bool durative_actions;           // Allow the use of "durative-action" in the domain definition
        bool durative_inequalities;      // Allows the use of inequalities to express a duration
        bool equality;                   // Allow the usage of "=" to compare objects
        bool existential_preconditions;  // Allows the use of "exists" in goals and preconditions
        bool negative_preconditions;     // Allows the use of "not" in preconditions
        bool numeric_fluents;            // Allows the use of the "function" block
        bool preferences;                // Allows for the use of preferences within problem definition (soft goals)
        bool strips;                     // Allows the usage of basic add/delete effects
        bool timed_initial_literals;     // Allows the use of timed initial literals in the problem definition
        bool typing;                     // Allows the usage for typing for objects
        bool universal_preconditions;    // Allows the use of "forall" in goal and preconditions
    };
}  // namespace grstapse