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
#include "grstapse/common/search/pruning_method_base.hpp"

namespace grstapse
{
    /**!
     * Pruning Method that is a conjunction of multiple pruning methods
     *
     * \tparam SearchNodeDeriv A derivative of SearchNodeBase
     */
    template <typename SearchNodeDeriv>
    class ConjunctivePruningMethod : public PruningMethodBase<SearchNodeDeriv>
    {
       public:
        //! Constructor
        explicit ConjunctivePruningMethod(
            const std::vector<std::shared_ptr<const PruningMethodBase<SearchNodeDeriv>>>& methods)
            : m_submethods(methods)
        {}

        //! \returns Whether to prune this node from the search
        [[nodiscard]] virtual bool operator()(const std::shared_ptr<const SearchNodeDeriv>& node) const final override
        {
            for(const auto method: m_submethods)
            {
                if(!method(node))
                {
                    return false;
                }
            }
            return true;
        }

       private:
        std::vector<std::shared_ptr<const PruningMethodBase<SearchNodeDeriv>>> m_submethods;
    };

}  // namespace grstapse