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
//// Robin Hood
#include <robin_hood/robin_hood.hpp>

namespace grstapse
{
    class PddlFunction;
    class PddlObject;
    class PddlRequirements;
    class PddlType;
    class PddlVariable;

    class PddlTask
    {
       public:
        //! Default constructor
        PddlTask();

        //! Sets the name of the domain
        void setDomainName(const std::string& name);
        //! \returns The name of the domain
        [[nodiscard]] const std::string& domainName() const;

        //! Sets the name of the problem
        void setProblemName(const std::string& name);
        //! \returns The name of the problem
        [[nodiscard]] const std::string& problemName() const;

        //! Set one of the requirement for a pddl task
        void setRequirement(const std::string& requirement);
        //! \returns A container for the requirements for this pddl
        [[nodiscard]] std::shared_ptr<const PddlRequirements> requirements() const;

        /**!
         * Adds a type from the pddl domain
         *
         * \param t The type
         * \param parent Its parent type
         */
        void addType(const std::string& type, std::string parent);
        //! \returns The specified type
        [[nodiscard]] std::shared_ptr<const PddlType> type(const std::string& type_name) const;
        //! \return The number of types
        [[nodiscard]] unsigned int numberOfTypes() const;

        //! Adds a predicate from the pddl domain
        void addPredicate(const std::string& name, const std::vector<PddlVariable>& parameters);
        //! Add a numeric function from the pddl domain
        void addFunction(const std::string& name, const std::vector<PddlVariable>& parameters);
        //! Adds an object function from the pddl domain
        void addFunction(const std::string& name,
                         const std::vector<PddlVariable>& parameters,
                         std::shared_ptr<const PddlType> return_type);
        //! \returns The specified function/predicate
        [[nodiscard]] std::shared_ptr<const PddlFunction> function(const std::string& name) const;
        //! \returns The number of predicates
        [[nodiscard]] unsigned int numberOfPredicates() const;
        //! \returns The number of numeric functions
        [[nodiscard]] unsigned int numberOfNumericFunctions() const;
        //! \returns The number of object functions
        [[nodiscard]] unsigned int numberOfObjectFunctions() const;
        //! \returns The total number of functions (predicates + numeric functions + object functions)
        [[nodiscard]] unsigned int totalNumberOfFunctions() const;

        //! Adds an object from the pddl problem
        void addObject(const std::string& name, const std::string& type);
        //! Adds a constant object from the pddl domain
        void addConstant(const std::string& name, const std::string& type);
        //! \returns The specified object
        [[nodiscard]] std::shared_ptr<const PddlObject> object(const std::string& name);
        //! \returns The number of objects
        [[nodiscard]] unsigned int numberOfObjects() const;
        //! \returns The number of constants
        [[nodiscard]] unsigned int numberOfConstants() const;
        //! \returns The total number of objects (objects + constants)
        [[nodiscard]] unsigned int totalNumberOfObjects() const;

       private:
        std::string m_domain_name;
        std::string m_problem_name;

        std::shared_ptr<PddlRequirements> m_requirements;

        robin_hood::unordered_map<std::string, std::shared_ptr<const PddlType>> m_types;
        std::shared_ptr<const PddlType> m_object_type;
        std::shared_ptr<const PddlType> m_boolean_type;
        std::shared_ptr<const PddlType> m_number_type;

        robin_hood::unordered_map<std::string, std::shared_ptr<const PddlFunction>> m_functions;
        robin_hood::unordered_map<std::string, std::shared_ptr<const PddlObject>> m_objects;
    };
}  // namespace grstapse