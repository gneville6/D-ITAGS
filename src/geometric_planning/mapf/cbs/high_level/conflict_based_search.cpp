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
#include "grstapse/geometric_planning/mapf/cbs/conflict_based_search.hpp"

// Local
#include "grstapse/common/utilities/time_keeper.hpp"
#include "grstapse/geometric_planning/mapf/cbs/conflict_based_search_parameters.hpp"
#include "grstapse/geometric_planning/mapf/cbs/high_level/conflict_base.hpp"
#include "grstapse/geometric_planning/mapf/cbs/high_level/constraint_tree_node.hpp"
#include "grstapse/geometric_planning/mapf/cbs/high_level/constraint_tree_node_root.hpp"
#include "grstapse/geometric_planning/mapf/cbs/high_level/edge_conflict.hpp"
#include "grstapse/geometric_planning/mapf/cbs/high_level/vertex_conflict.hpp"
#include "grstapse/geometric_planning/mapf/cbs/high_level/vertex_constraint.hpp"
#include "grstapse/geometric_planning/mapf/cbs/low_level/space_time_a_star_with_constraints.hpp"

namespace grstapse
{
    ConflictBaseSearch::ConflictBaseSearch(const std::shared_ptr<const ConflictBasedSearchParameters>& parameters)
        : Base(parameters)
    {}

    std::shared_ptr<ConstraintTreeNodeBase> ConflictBaseSearch::createRootNode()
    {
        auto cbs_parameters           = std::dynamic_pointer_cast<const ConflictBasedSearchParameters>(m_parameters);
        const unsigned int num_robots = cbs_parameters->numberOfRobots();
        return std::make_shared<ConstraintTreeNodeRoot>(num_robots, cbs_parameters->cost_type);
    }

    SearchResults<ConstraintTreeNodeBase, ConflictBasedSearchStatistics> ConflictBaseSearch::searchFromNode(
        const std::shared_ptr<ConstraintTreeNodeBase>& root)
    {
        auto cbs_parameters           = std::dynamic_pointer_cast<const ConflictBasedSearchParameters>(m_parameters);
        const unsigned int num_robots = cbs_parameters->numberOfRobots();
        if(!computeLowLevelSolution(root))
        {
            return SearchResults<ConstraintTreeNodeBase, ConflictBasedSearchStatistics>(nullptr, Base::m_statistics);
        }
        Base::m_statistics->incrementNumberOfHighLevelNodesGenerated();
        m_open.push(root->id(), root);

        while(!m_open.empty())
        {
            std::shared_ptr<ConstraintTreeNodeBase> base = m_open.pop();

            std::unique_ptr<const ConflictBase> conflict = base->getFirstConflict();
            if(conflict == nullptr)
            {
                return SearchResults<ConstraintTreeNodeBase, ConflictBasedSearchStatistics>(base, Base::m_statistics);
            }
            base->setStatus(SearchNodeStatus::e_closed);

            robin_hood::unordered_map<unsigned int, std::shared_ptr<ConstraintBase>> constraints =
                conflict->createConstraints();
            for(const auto& [robot, constraint]: constraints)
            {
                auto child = std::make_shared<ConstraintTreeNode>(num_robots, cbs_parameters->cost_type, base);
                child->setConstraint(robot, constraint);
                Base::m_statistics->incrementNumberOfHighLevelNodesGenerated();
                if(computeLowLevelSolution(child, robot))
                {
                    m_open.push(child->id(), child);
                    child->setStatus(SearchNodeStatus::e_open);
                }
                Base::m_statistics->incrementNumberOfHighLevelNodesEvaluated();
            }
        }

        return SearchResults<ConstraintTreeNodeBase, ConflictBasedSearchStatistics>(nullptr, Base::m_statistics);
    }

    bool ConflictBaseSearch::computeLowLevelSolution(const std::shared_ptr<ConstraintTreeNodeBase>& node)
    {
        auto cbs_parameters = std::dynamic_pointer_cast<const ConflictBasedSearchParameters>(m_parameters);
        for(unsigned int i = 0, num_robots = cbs_parameters->numberOfRobots(); i < num_robots; ++i)
        {
            if(!computeLowLevelSolution(node, i))
            {
                return false;
            }
        }
        return true;
    }

    bool ConflictBaseSearch::computeLowLevelSolution(const std::shared_ptr<ConstraintTreeNodeBase>& node,
                                                     unsigned int robot)
    {
        auto cbs_parameters = std::dynamic_pointer_cast<const ConflictBasedSearchParameters>(m_parameters);
        std::shared_ptr<const SpaceTimeAStarParameters> low_level_parameters;
        if(m_parameters->has_timeout)
        {
            low_level_parameters = std::make_shared<const SpaceTimeAStarParameters>(
                cbs_parameters->low_level_timer_name,
                true,
                m_parameters->timeout - TimeKeeper::instance().time(m_parameters->timer_name));
        }
        else
        {
            low_level_parameters =
                std::make_shared<const SpaceTimeAStarParameters>(cbs_parameters->low_level_timer_name);
        }
        SpaceTimeAStarWithConstraints low_level(low_level_parameters,
                                                cbs_parameters->map(),
                                                cbs_parameters->initialStates()[robot],
                                                cbs_parameters->goalStates()[robot],
                                                node->constraints(robot));
        SearchResults<TemporalGridCellNode, SearchStatisticsCommon> result = low_level.search();
        std::shared_ptr<SearchStatisticsCommon> low_level_statistics       = result.statistics();
        Base::m_statistics->incrementNumberOfLowLevelNodesGenerated(low_level_statistics->numberOfNodesGenerated());
        Base::m_statistics->incrementNumberOfLowLevelNodesEvaluated(low_level_statistics->numberOfNodesEvaluated());
        Base::m_statistics->incrementNumberOfLowLevelNodesExpanded(low_level_statistics->numberOfNodesExpanded());
        if(!result.foundGoal())
        {
            return false;
        }
        node->setLowLevelSolution(robot, result.goal());
        return true;
    }
}  // namespace grstapse