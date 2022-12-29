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
#include <queue>
// External
#include <nlohmann/json.hpp>
#include <ompl/base/StateSpace.h>
#include <ompl/base/StateValidityChecker.h>
// Local
#include "grstapse/geometric_planning/environment_base.hpp"

namespace grstapse
{
    /*
     * https://ompl.kavrakilab.org/classompl_1_1Grid.html
     * https://ompl.kavrakilab.org/classPolyWorld.html
     */

    // Forward Declarations
    enum class OmplStateSpaceType : uint8_t;

    enum class OmplEnvironmentType : uint8_t
    {
        e_unknown = 0,
        e_pgm
    };
    NLOHMANN_JSON_SERIALIZE_ENUM(OmplEnvironmentType,
                                 {{OmplEnvironmentType::e_unknown, "unknown"}, {OmplEnvironmentType::e_pgm, "pgm"}});

    /**!
     * Abstract base class for the environment that is to be used with motion planners from OMPL
     *
     * Derivative classes are used to setup the state space and determine if states are valid
     */
    class OmplEnvironment
        : public ompl::base::StateValidityChecker
        , public EnvironmentBase
    {
       public:
        //! Deserialize from json
        [[nodiscard]] static std::shared_ptr<OmplEnvironment> deserializeFromJson(const nlohmann::json& j);

        //! \copydoc ompl::base::StateValidityChecker
        [[nodiscard]] virtual bool isValid(const ompl::base::State* state) const override = 0;

        //! \returns The state space for this environment
        [[nodiscard]] inline const std::shared_ptr<ompl::base::StateSpace>& stateSpace() const;

        //! \returns The type of ompl environment represented
        [[nodiscard]] inline OmplEnvironmentType environmentType() const;

        //! \returns The type of state space represented
        [[nodiscard]] inline OmplStateSpaceType stateSpaceType() const;

       protected:
        //! Default Constructor
        OmplEnvironment(OmplEnvironmentType environment_type, OmplStateSpaceType state_space_type);

        OmplEnvironmentType m_environment_type;
        OmplStateSpaceType m_state_space_type;
        std::shared_ptr<ompl::base::StateSpace> m_state_space;
    };

    // Inline Functions
    const std::shared_ptr<ompl::base::StateSpace>& OmplEnvironment::stateSpace() const
    {
        assert(m_state_space);
        return m_state_space;
    }

    OmplEnvironmentType OmplEnvironment::environmentType() const
    {
        return m_environment_type;
    }

    OmplStateSpaceType OmplEnvironment::stateSpaceType() const
    {
        return m_state_space_type;
    }
}  // namespace grstapse