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
#include "grstapse/common/search/best_first_search_parameters.hpp"

namespace grstapse
{
    /**!
     * \brief Container for parameters for A* Epsilon
     */
    struct FocalAStarParameters : public BestFirstSearchParameters
    {
       public:
        /**!
         * Constructor
         *
         * \param timer_name The name of the timer
         * \param w Suboptimality factor
         * \param rebuild Whether to rebuild the focal set upon each iteration. If false then the focal set will be
         *                merely updated.
         * \param has_timeout Whether the search has a timeout
         * \param timeout The timeout for the search
         * \param save_pruned_nodes
         * \param save_closed_nodes
         */
        FocalAStarParameters(const std::string& timer_name,
                             float w                = 1.1,
                             bool rebuild           = false,
                             bool has_timeout       = false,
                             float timeout          = std::numeric_limits<float>::max(),
                             bool save_pruned_nodes = false,
                             bool save_closed_nodes = false)
            : BestFirstSearchParameters{.has_timeout       = has_timeout,
                                        .timeout           = timeout,
                                        .timer_name        = timer_name,
                                        .save_pruned_nodes = save_pruned_nodes,
                                        .save_closed_nodes = save_closed_nodes}
            , w(w)
            , rebuild(rebuild)
        {}

        float w;
        bool rebuild;
    };

}  // namespace grstapse