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
#include <mutex>
// External
#include <nlohmann/json.hpp>
// Local
#include "grstapse/common/utilities/noncopyable.hpp"
#include "grstapse/geometric_planning/configuration_base.hpp"

namespace grstapse
{
    // Forward Declaration
    class Species;

    /**!
     * Abstract base class for the environment
     */
    class EnvironmentBase : public Noncopyable
    {
       public:
        //! Load an environment from one or more files (filepaths are encoded in json)
        [[nodiscard]] static std::shared_ptr<EnvironmentBase> deserializeFromJson(const nlohmann::json& j);

        //! \returns An overestimate of the longest path through the environment
        [[nodiscard]] virtual float longestPath() const = 0;

        //! Locks a mutex
        inline void lock();

        //! Unlocks a mutex
        inline void unlock();

        //! Set the species of the robot for the next so many computations
        inline void setSpecies(const std::shared_ptr<const Species>& species);

        //! \returns The type of configurations that can be used with this environment
        [[nodiscard]] inline ConfigurationType configurationType() const;

       protected:
        //! Constructor
        explicit EnvironmentBase(ConfigurationType configuration_type);

        ConfigurationType m_configuration_type;
        std::shared_ptr<const Species> m_species;
        std::mutex m_mutex;
    };

    /**!
     * Concept to force a type to derive from EnvironmentBase
     *
     * \tparam T Derivative of EnvironmentBase
     */
    template <typename T>
    concept EnvironmentDeriv = std::derived_from<T, EnvironmentBase>;

    // Inline Functions
    void EnvironmentBase::lock()
    {
        m_mutex.lock();
    }

    void EnvironmentBase::unlock()
    {
        m_mutex.unlock();
    }

    ConfigurationType EnvironmentBase::configurationType() const
    {
        return m_configuration_type;
    }

    void EnvironmentBase::setSpecies(const std::shared_ptr<const Species>& species)
    {
        m_species = species;
    }

}  // namespace grstapse