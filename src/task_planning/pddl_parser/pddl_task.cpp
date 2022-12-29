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
#include "grstapse/task_planning/pddl_parser/pddl_task.hpp"

// External
#include <fmt/format.h>

// Local
#include "grstapse/common/utilities/error.hpp"
#include "grstapse/task_planning/pddl_parser/pddl_function.hpp"
#include "grstapse/task_planning/pddl_parser/pddl_object.hpp"
#include "grstapse/task_planning/pddl_parser/pddl_requirements.hpp"
#include "grstapse/task_planning/pddl_parser/pddl_type.hpp"
#include "grstapse/task_planning/pddl_parser/pddl_variable.hpp"

namespace grstapse
{
    PddlTask::PddlTask()
        : m_requirements(std::make_shared<PddlRequirements>())
    {
        m_object_type = m_types["#object"] = std::make_shared<PddlType>("#object", nullptr);
        m_boolean_type = m_types["#boolean"] = std::make_shared<PddlType>("#boolean", nullptr);
        m_number_type = m_types["#number"] = std::make_shared<PddlType>("#number", nullptr);
    }

    void PddlTask::setDomainName(const std::string& name)
    {
        m_domain_name = name;
    }

    const std::string& PddlTask::domainName() const
    {
        return m_domain_name;
    }

    void PddlTask::setProblemName(const std::string& name)
    {
        m_problem_name = name;
    }

    const std::string& PddlTask::problemName() const
    {
        return m_problem_name;
    }

    void PddlTask::setRequirement(const std::string& requirement)
    {
        m_requirements->set(requirement);
    }

    std::shared_ptr<const PddlRequirements> PddlTask::requirements() const
    {
        return m_requirements;
    }

    void PddlTask::addType(const std::string& t, std::string parent)
    {
        if(parent == "object")
        {
            parent = "#object";
        }

        if(!m_types.contains(parent))
        {
            throw createLogicError(
                fmt::format("'{0:s}' is not a previously define type and so cannot be the parent of '{1:s}'",
                            parent,
                            t));
        }
        std::shared_ptr<const PddlType> parent_type = m_types[parent];
        m_types[t]                                  = std::make_shared<PddlType>(t, parent_type);
    }

    std::shared_ptr<const PddlType> PddlTask::type(const std::string& type_name) const
    {
        if(m_types.contains(type_name))
        {
            return m_types.at(type_name);
        }
        throw createLogicError(fmt::format("'{0:s}' is not a type in this domain", type_name));
    }

    unsigned int PddlTask::numberOfTypes() const
    {
        // Ignores default "#object", "#boolean", and "#number"
        return m_types.size() - 3;
    }
    void PddlTask::addPredicate(const std::string& name, const std::vector<PddlVariable>& parameters)
    {
        m_functions[name] = std::make_shared<PddlFunction>(name, parameters, m_boolean_type);
    }

    void PddlTask::addFunction(const std::string& name, const std::vector<PddlVariable>& parameters)
    {
        m_functions[name] = std::make_shared<PddlFunction>(name, parameters, m_number_type);
    }

    void PddlTask::addFunction(const std::string& name,
                               const std::vector<PddlVariable>& parameters,
                               std::shared_ptr<const PddlType> return_type)
    {
        m_functions[name] = std::make_shared<PddlFunction>(name, parameters, return_type);
    }

    std::shared_ptr<const PddlFunction> PddlTask::function(const std::string& name) const
    {
        if(m_functions.contains(name))
        {
            return m_functions.at(name);
        }
        throw createLogicError(fmt::format("'{0:s}' is not a function in this domain", name));
    }

    unsigned int PddlTask::numberOfPredicates() const
    {
        unsigned int rv = 0;
        for(auto [name, function]: m_functions)
        {
            if(function->returnType() == m_boolean_type)
            {
                ++rv;
            }
        }
        return rv;
    }

    unsigned int PddlTask::numberOfNumericFunctions() const
    {
        unsigned int rv = 0;
        for(auto [name, function]: m_functions)
        {
            if(function->returnType() == m_number_type)
            {
                ++rv;
            }
        }
        return rv;
    }

    unsigned int PddlTask::numberOfObjectFunctions() const
    {
        unsigned int rv = 0;
        for(auto [name, function]: m_functions)
        {
            if(function->returnType() != m_boolean_type && function->returnType() != m_number_type)
            {
                ++rv;
            }
        }
        return rv;
    }

    unsigned int PddlTask::totalNumberOfFunctions() const
    {
        return m_functions.size();
    }

    void PddlTask::addObject(const std::string& name, const std::string& type_name)
    {
        std::shared_ptr<const PddlType> t = type(type_name);
        m_objects[name]                   = std::make_shared<PddlObject>(name, t, false);
    }

    void PddlTask::addConstant(const std::string& name, const std::string& type_name)
    {
        std::shared_ptr<const PddlType> t = type(type_name);
        m_objects[name]                   = std::make_shared<PddlObject>(name, t, true);
    }

    std::shared_ptr<const PddlObject> PddlTask::object(const std::string& name)
    {
        if(m_objects.contains(name))
        {
            return m_objects.at(name);
        }
        throw createLogicError(fmt::format("'{0:s}' is not a object in this domain", name));
    }

    unsigned int PddlTask::numberOfObjects() const
    {
        unsigned int rv = 0;
        for(auto [name, object]: m_objects)
        {
            if(!object->constant())
            {
                ++rv;
            }
        }
        return rv;
    }

    unsigned int PddlTask::numberOfConstants() const
    {
        unsigned int rv = 0;
        for(auto [name, object]: m_objects)
        {
            if(object->constant())
            {
                ++rv;
            }
        }
        return rv;
    }

    unsigned int PddlTask::totalNumberOfObjects() const
    {
        return m_objects.size();
    }
}  // namespace grstapse