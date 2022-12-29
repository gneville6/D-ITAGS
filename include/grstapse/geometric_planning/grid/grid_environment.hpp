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
// External
// Local
#include "grstapse/geometric_planning/environment_base.hpp"
#include "grstapse/geometric_planning/grid/grid_map.hpp"

namespace grstapse
{
    // Forward Declarations

    /**!
     *
     */
    class GridEnvironment : public EnvironmentBase
    {
       public:
        //! Constructor
        GridEnvironment(const GridMap& map)
            : EnvironmentBase(ConfigurationType::e_graph)
            , m_map(map)
        {}

        float longestPath() const override;

       private:
        GridMap m_map;
    };

}  // namespace grstapse