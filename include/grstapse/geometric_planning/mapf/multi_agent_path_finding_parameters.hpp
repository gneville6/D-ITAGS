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

#include "grstapse/geometric_planning/grid/grid_cell.hpp"
#include "grstapse/geometric_planning/grid/grid_map.hpp"

namespace grstapse
{
    //! Container for the parameters for a MAPF solve
    class MultiAgentPathFindingParameters
    {
       public:
        //! Constructor
        MultiAgentPathFindingParameters(const std::shared_ptr<const GridMap>& map,
                                        const std::vector<std::shared_ptr<const GridCell>>& initial_states,
                                        const std::vector<std::shared_ptr<const GridCell>>& goal_states);

        //! \returns The map
        [[nodiscard]] inline std::shared_ptr<const GridMap> map() const noexcept;

        //! \returns A list of the initial states
        [[nodiscard]] inline const std::vector<std::shared_ptr<const GridCell>>& initialStates() const noexcept;

        //! \returns A list of the goal states
        [[nodiscard]] inline const std::vector<std::shared_ptr<const GridCell>>& goalStates() const noexcept;

        //! \returns The number of robots
        [[nodiscard]] inline unsigned int numberOfRobots() const noexcept;

       private:
        std::shared_ptr<const GridMap> m_map;
        std::vector<std::shared_ptr<const GridCell>> m_initial_states;
        std::vector<std::shared_ptr<const GridCell>> m_goal_states;
    };

    // Inline functions
    std::shared_ptr<const GridMap> MultiAgentPathFindingParameters::map() const noexcept
    {
        return m_map;
    }
    const std::vector<std::shared_ptr<const GridCell>>& MultiAgentPathFindingParameters::initialStates() const noexcept
    {
        return m_initial_states;
    }
    const std::vector<std::shared_ptr<const GridCell>>& MultiAgentPathFindingParameters::goalStates() const noexcept
    {
        return m_goal_states;
    }
    unsigned int MultiAgentPathFindingParameters::numberOfRobots() const noexcept
    {
        return m_initial_states.size();
    }

}  // namespace grstapse