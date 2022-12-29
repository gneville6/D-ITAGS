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
#include "grstapse/task_planning/pddl_parser/pddl_durative_action.hpp"

// Local
#include "grstapse/task_planning/pddl_parser/pddl_variable.hpp"

namespace grstapse
{
    PddlDurativeAction::PddlDurativeAction(const std::string& name,
                                           const std::vector<PddlVariable>& parameters,
                                           std::shared_ptr<const PddlDuration> duration,
                                           std::shared_ptr<const PddlCondition> conditions,
                                           std::shared_ptr<const PddlEffect> effects)
        : m_name(name)
        , m_id(s_next_id++)
        , m_parameters(parameters)
        , m_duration(duration)
        , m_conditions(conditions)
        , m_effects(effects)
    {}

    const std::string& PddlDurativeAction::name() const
    {
        return m_name;
    }

    unsigned int PddlDurativeAction::id() const
    {
        return m_id;
    }

    const std::vector<PddlVariable>& PddlDurativeAction::parameters() const
    {
        return m_parameters;
    }

    std::shared_ptr<const PddlDuration> PddlDurativeAction::duration() const
    {
        return m_duration;
    }

    std::shared_ptr<const PddlCondition> PddlDurativeAction::conditions() const
    {
        return m_conditions;
    }

    std::shared_ptr<const PddlEffect> PddlDurativeAction::effects() const
    {
        return m_effects;
    }
    std::ostream& PddlDurativeAction::print(std::ostream& o) const
    {
        // TODO(Andrew): Implement
        throw std::logic_error("Not Implemented");
        return o;
    }
}  // namespace grstapse