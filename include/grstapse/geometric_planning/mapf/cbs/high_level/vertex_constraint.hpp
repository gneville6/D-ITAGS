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
#include <utility>

// Local
#include "grstapse/geometric_planning/mapf/cbs/high_level/constraint_base.hpp"
#include "grstapse/geometric_planning/mapf/cbs/low_level/temporal_grid_cell.hpp"

namespace grstapse
{
    //! A constraint representing that a robot can not occupy the vertex at (x, y) at time t
    class VertexConstraint
        : public ConstraintBase
        , public TemporalGridCell
    {
       public:
        VertexConstraint(unsigned int time, unsigned int x, unsigned int y);

        using TemporalGridCell::operator==;
        using TemporalGridCell::operator!=;

        //! \copydoc ConstraintBase
        size_t hash() const override;
    };
}  // namespace grstapse