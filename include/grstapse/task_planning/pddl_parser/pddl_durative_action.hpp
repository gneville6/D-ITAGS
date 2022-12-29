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

// Local
#include "grstapse/task_planning/pddl_parser/pddl_base.hpp"

namespace grstapse
{
    class PddlCondition;
    class PddlDuration;
    class PddlEffect;
    class PddlVariable;

    class PddlDurativeAction : public PddlBase
    {
       public:
        PddlDurativeAction(const std::string& name,
                           const std::vector<PddlVariable>& parameters,
                           std::shared_ptr<const PddlDuration> duration,
                           std::shared_ptr<const PddlCondition> conditions,
                           std::shared_ptr<const PddlEffect> effects);

        const std::string& name() const;
        unsigned int id() const;
        const std::vector<PddlVariable>& parameters() const;
        std::shared_ptr<const PddlDuration> duration() const;
        std::shared_ptr<const PddlCondition> conditions() const;
        std::shared_ptr<const PddlEffect> effects() const;
        std::ostream& print(std::ostream& o) const override;

       private:
        std::string m_name;
        unsigned int m_id;
        std::shared_ptr<const PddlDuration> m_duration;
        std::vector<PddlVariable> m_parameters;
        std::shared_ptr<const PddlCondition> m_conditions;
        std::shared_ptr<const PddlEffect> m_effects;

        unsigned int s_next_id;
    };
}  // namespace grstapse