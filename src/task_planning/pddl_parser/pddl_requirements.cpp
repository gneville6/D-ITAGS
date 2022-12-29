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
#include "grstapse/task_planning/pddl_parser/pddl_requirements.hpp"

// External
#include <fmt/format.h>

// Local
#include "grstapse/common/utilities/error.hpp"

namespace grstapse
{
    PddlRequirements::PddlRequirements()
        : typing(false)
    {}

    void PddlRequirements::set(const std::string& requirement)
    {
        if(requirement == "adl")
        {
            strips                    = true;
            typing                    = true;
            disjunctive_preconditions = true;
            equality                  = true;
            // quantified-preconditions
            existential_preconditions = true;
            universal_preconditions   = true;
            conditional_effects       = true;
        }
        else if(requirement == "conditional-effects")
        {
            conditional_effects = true;
        }
        else if(requirement == "constraints")
        {
            throw createLogicError("We currently cannot handle the 'constraints' requirement");
            constraints = true;
        }
        else if(requirement == "continuous-effects")
        {
            throw createLogicError("We currently cannot handle the 'continuous-effects' requirement");
            continuous_effects = true;
        }
        else if(requirement == "derived-predicates")
        {
            throw createLogicError("We currently cannot handle the 'derived-predicate' requirement");
            derived_predicates = true;
        }
        else if(requirement == "disjunctive-preconditions")
        {
            disjunctive_preconditions = true;
        }
        else if(requirement == "durative-actions")
        {
            durative_actions = true;
        }
        else if(requirement == "durative-inequalities")
        {
            throw createLogicError("We currently cannot handle the 'durative-inequalities' requirement");
            durative_inequalities = true;
        }
        else if(requirement == "existential-preconditions")
        {
            existential_preconditions = true;
        }
        else if(requirement == "equality")
        {
            equality = true;
        }
        else if(requirement == "negative-preconditions")
        {
            negative_preconditions = true;
        }
        else if(requirement == "numeric-fluents")
        {
            throw createLogicError("We currently cannot handle the 'numeric-fluents' requirement");
            numeric_fluents = true;
        }
        else if(requirement == "preferences")
        {
            throw createLogicError("We currently cannot handle the 'preferences' requirement");
            preferences = true;
        }
        else if(requirement == "quantified-preconditions")
        {
            existential_preconditions = true;
            universal_preconditions   = true;
        }
        else if(requirement == "strips")
        {
            strips = true;
        }
        else if(requirement == "timed-initial-literals")
        {
            timed_initial_literals = true;
        }
        else if(requirement == "typing")
        {
            typing = true;
        }
        else if(requirement == "universal-preconditions")
        {
            universal_preconditions = true;
        }
        else
        {
            throw createLogicError(fmt::format("Unknown requirement: '{}'", requirement));
        }
    }

    std::ostream& PddlRequirements::print(std::ostream& os) const
    {
        os << "( :requirements";

        bool quanitied_preconditions = existential_preconditions && universal_preconditions;
        bool adl =
            strips && typing && disjunctive_preconditions && equality && quanitied_preconditions && conditional_effects;

        if(adl)
        {
            os << " :adl";
        }
        if(!adl && conditional_effects)
        {
            os << " :conditional-effects";
        }
        if(constraints)
        {
            os << " :constraints";
        }
        if(continuous_effects)
        {
            os << " :continuous-effects";
        }
        if(derived_predicates)
        {
            os << " :derived-predicates";
        }
        if(!adl && disjunctive_preconditions)
        {
            os << " :disjunctive-preconditions";
        }
        if(durative_actions)
        {
            os << " :durative-actions";
        }
        if(durative_inequalities)
        {
            os << " :durative-inequalities";
        }
        if(!adl && equality)
        {
            os << " :equality";
        }
        if(!quanitied_preconditions && existential_preconditions)
        {
            os << " :existential-preconditions";
        }
        if(negative_preconditions)
        {
            os << " :negative-preconditions";
        }
        if(numeric_fluents)
        {
            os << " :numeric-fluents";
        }
        if(preferences)
        {
            os << " :preferences";
        }
        if(!adl && quanitied_preconditions)
        {
            os << " :quantified-preconditions";
        }
        if(timed_initial_literals)
        {
            os << " :timed-initial-literals";
        }
        if(!adl && typing)
        {
            os << " :typing";
        }
        if(!quanitied_preconditions && universal_preconditions)
        {
            os << " :universal-preconditions";
        }

        os << " )\n";
        return os;
    }
}  // namespace grstapse