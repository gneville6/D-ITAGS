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

// Local
#include "grstapse/geometric_planning/mapf/cbs/high_level/conflict_base.hpp"
#include "grstapse/geometric_planning/mapf/cbs/low_level/temporal_grid_cell.hpp"

namespace grstapse
{
    /**!
     * A conflict between two or more robots' low level plans on a vertex
     */
    class VertexConflict
        : public ConflictBase
        , public TemporalGridCell
    {
       public:
        /**!
         * Constructor
         *
         * \param agents
         * \param time
         * \param x
         * \param y
         */
        VertexConflict(const std::array<unsigned int, 2>& agents, unsigned int time, unsigned int x, unsigned y);

        //! \returns Constraints for the two robots that are part of the conflict
        robin_hood::unordered_map<unsigned int, std::shared_ptr<ConstraintBase>> createConstraints() const override;
    };

}  // namespace grstapse