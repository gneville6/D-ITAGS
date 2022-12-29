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

// Local
#include "grstapse/geometric_planning/ompl/ompl_configuration.hpp"

namespace grstapse
{
    // Forward Declaration
    class Se3StateOmplConfiguration;

    /**!
     * A task configuration from SE(3) that works with an ompl motion planner
     */
    class Se3OmplConfiguration : public OmplConfiguration
    {
       public:
        //! Deserializes from json
        [[nodiscard]] static std::shared_ptr<const Se3OmplConfiguration> deserializeFromJson(const nlohmann::json& j);

        //! \copydoc ConfigurationBase
        [[nodiscard]] virtual float euclideanDistance(
            const std::shared_ptr<const ConfigurationBase>& rhs) const final override;

       protected:
        //! Constructor
        explicit Se3OmplConfiguration(OmplGoalType goal_type);

        //! Minimum euclidean distance from this task configuration base to \p rhs
        [[nodiscard]] virtual float euclideanDistance(
            const std::shared_ptr<const Se3StateOmplConfiguration>& rhs) const = 0;
    };
}  // namespace grstapse