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
#include <array>

// External
#include <robin_hood/robin_hood.hpp>

namespace grstapse
{
    // Forward Declarations
    class ConstraintBase;

    /**!
     * Abstract base class for a conflict
     *
     * \note Can only be derived from
     */
    class ConflictBase
    {
       public:
        //! \returns The ids for the two agents in this conflict
        [[nodiscard]] inline const std::array<unsigned int, 2>& agents() const;

        //! \returns The id for the first agent in this conflict
        [[nodiscard]] inline unsigned int agent1() const;

        //! \returns The id for the seconds agent in this conflict
        [[nodiscard]] inline unsigned int agent2() const;

        //! \returns A map of constraints for the robots involved in the conflict
        virtual robin_hood::unordered_map<unsigned int, std::shared_ptr<ConstraintBase>> createConstraints() const = 0;

       protected:
        /**!
         * Constructor
         *
         * \param agents
         */
        explicit ConflictBase(const std::array<unsigned int, 2>& agents);

        std::array<unsigned int, 2> m_agents;
    };
}  // namespace grstapse