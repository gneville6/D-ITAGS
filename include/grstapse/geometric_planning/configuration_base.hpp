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
    enum class ConfigurationType : uint8_t
    {
        e_unknown = 0,
        e_ompl,
        e_graph
    };
    NLOHMANN_JSON_SERIALIZE_ENUM(ConfigurationType,
                                 {{ConfigurationType::e_unknown, "unknown"},
                                  {ConfigurationType::e_ompl, "ompl"},
                                  {ConfigurationType::e_graph, "graph"}});

    /**!
     *  Abstract base class for the initial/terminal configuration for a task and the initial configuration for a robot
     *
     *  \see Task
     *  \see Robot
     */
    class ConfigurationBase
    {
       public:
        //! Deserializes from json
        static std::shared_ptr<const ConfigurationBase> deserializeFromJson(const nlohmann::json& j);

        //! \returns The minimum euclidean distance from this task configuration base to \p rhs
        [[nodiscard]] virtual float euclideanDistance(const std::shared_ptr<const ConfigurationBase>& rhs) const = 0;

        //! \returns Whether \rhs is equal to this TaskConfigurationBase
        [[nodiscard]] virtual bool isEqual(const std::shared_ptr<const ConfigurationBase>& rhs) const = 0;

        //! \returns
        [[nodiscard]] inline ConfigurationType configurationType() const;

       protected:
        //! Constructor
        explicit ConfigurationBase(ConfigurationType type);

        ConfigurationType m_type;
    };

    /**!
     * Concept to force a type to derive from ConfigurationBase
     *
     * \tparam T Derivative of ConfigurationBase
     */
    template <typename T>
    concept ConfigurationDeriv = std::derived_from<T, ConfigurationBase>;

    // Inline Functions
    ConfigurationType ConfigurationBase::configurationType() const
    {
        return m_type;
    }
}  // namespace grstapse