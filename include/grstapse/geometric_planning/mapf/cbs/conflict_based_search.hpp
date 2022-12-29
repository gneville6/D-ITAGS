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
#include "grstapse/common/search/search_algorithm_base.hpp"
#include "grstapse/common/utilities/mutable_priority_queue/mutable_priority_queue.hpp"
#include "grstapse/geometric_planning/mapf/cbs/conflict_based_search_statistics.hpp"
#include "grstapse/geometric_planning/mapf/cbs/high_level/constraint_tree_node_base.hpp"

namespace grstapse
{
    // Forward Declarations
    class ConflictBasedSearchParameters;
    class ConstraintTreeNodeRoot;
    class ConstraintTreeNode;

    /**!
     * Implementation of the Conflict-Based Search (CBS) algorithm.
     *
     * This algorithm provides an optimal solution to the multi-agent
     * pathfinding (MAPF) problem. It works as a two-level search. On
     * the lower level, A* is used to find paths for individual agents.
     * On the higher level, a tree search is used to resolve conflicts
     * between individual agents.
     *
     * \cite Guni Sharon, Roni Stern, Ariel Felner, Nathan R. Sturtevant:
     *       "Conflict-based search for optimal multi-agent pathfinding".
     *       Artif. Intell. 219:40-66 (2015)
     *
     * \ref https://github.com/whoenig/libMultiRobotPlanning
     */
    class ConflictBaseSearch : public SearchAlgorithmBase<ConstraintTreeNodeBase, ConflictBasedSearchStatistics>
    {
        using Base = SearchAlgorithmBase<ConstraintTreeNodeBase, ConflictBasedSearchStatistics>;

       public:
        /**!
         * Constructor
         *
         * \param parameters Parameters for solving a MAPF problem with Conflict-Based Search
         */
        explicit ConflictBaseSearch(const std::shared_ptr<const ConflictBasedSearchParameters>& parameters);

        //! \copydoc SearchAlgorithmBase
        std::shared_ptr<ConstraintTreeNodeBase> createRootNode() override;

        //! \copydoc SearchAlgorithmBase
        SearchResults<ConstraintTreeNodeBase, ConflictBasedSearchStatistics> searchFromNode(
            const std::shared_ptr<ConstraintTreeNodeBase>& node) override;

       private:
        /**!
         * Runs a series of A* searches on a temporal grid
         *
         * \param node A node from the Conflict Tree which contains constraints used to resolve conflicts from previous
         *             low-level searches
         * \returns Whether the low-level searches were successful
         */
        bool computeLowLevelSolution(const std::shared_ptr<ConstraintTreeNodeBase>& node);

        /**!
         * \brief Computes a low level trajectory for a single robot
         *
         * \param node A node from the Conflict Tree which contains constraints used to resolve conflicts from previous
         *             low-level searches
         * \param robot The id for the robot to compute the low level trajectory for
         *
         * \returns Whether the low-level search was successful
         */
        bool computeLowLevelSolution(const std::shared_ptr<ConstraintTreeNodeBase>& node, unsigned int robot);

        /**!
         * \returns Whether the final position of a robot violates a vertex constraint
         */
        bool checkPostRouteViolation(
            const robin_hood::unordered_set<std::shared_ptr<const ConstraintBase>>& constraints,
            std::shared_ptr<const TemporalGridCellNode> goal) const;

        MutablePriorityQueue<unsigned int, unsigned int, ConstraintTreeNodeBase> m_open;
    };
}  // namespace grstapse