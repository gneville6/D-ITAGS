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
#include "grstapse/common/search/a_star/a_star.hpp"
#include "grstapse/common/search/best_first_search_parameters.hpp"
#include "grstapse/common/search/undirected_graph/undirected_graph_a_star_search_node.hpp"
#include "grstapse/geometric_planning/graph/point/point_graph_configuration.hpp"
#include "grstapse/geometric_planning/graph/point/point_graph_environment.hpp"

namespace grstapse
{
    /**!
     * An A* search on a point graph
     */
    class PointGraphAStar : public AStar<UndirectedGraphAStarSearchNode<PointGraphConfiguration>>
    {
        using SearchNode = UndirectedGraphAStarSearchNode<PointGraphConfiguration>;
        using Base       = AStar<SearchNode>;

       public:
        //! Constructor
        PointGraphAStar(const std::shared_ptr<const BestFirstSearchParameters>& parameters,
                        const std::shared_ptr<const PointGraphConfiguration>& root,
                        const std::shared_ptr<PointGraphEnvironment>& graph,
                        const AStarFunctors<SearchNode>& functors)
            : Base(parameters, functors)
            , m_root(std::make_shared<SearchNode>(graph->findVertex(root)))
        {}

        std::shared_ptr<SearchNode> createRootNode() final override
        {
            return m_root;
        }

       private:
        std::shared_ptr<SearchNode> m_root;
    };

}  // namespace grstapse