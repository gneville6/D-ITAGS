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
#include <ostream>

// Local
#include "grstapse/common/search/search_statistics_base.hpp"

namespace grstapse
{
    /**!
     * The statistics from a graph/tree search
     */
    class SearchStatisticsCommon : public SearchStatisticsBase
    {
       public:
        //! \brief Constructor
        SearchStatisticsCommon();

        //! \returns The # of nodes for which successors were generated
        [[nodiscard]] inline unsigned int numberOfNodesExpanded() const;

        //! \returns # of nodes for which heuristic was computed
        [[nodiscard]] inline unsigned int numberOfNodesEvaluated() const;

        //! \returns # of nodes created in total
        [[nodiscard]] inline unsigned int numberOfNodesGenerated() const;

        //! \returns # of *closed* nodes which were reopened
        [[nodiscard]] inline unsigned int numberOfNodesReopened() const;

        //! \returns # of nodes for which no successor could be generated
        [[nodiscard]] inline unsigned int numberOfDeadendNodes() const;

        //! \returns # of nodes pruned
        [[nodiscard]] inline unsigned int numberOfNodesPruned() const;

        //! \brief Sets the # of nodes for which successors were generated
        inline void incrementNodesExpanded(unsigned int inc = 1);

        //! \brief Sets the # of nodes for which heuristic was computed
        inline void incrementNodesEvaluated(unsigned int inc = 1);

        //! \brief Sets the # of nodes created in total
        inline void incrementNodesGenerated(unsigned int inc = 1);

        //! \brief Sets the # of *closed* nodes which were reopened
        inline void incrementNodesReopened(unsigned int inc = 1);

        //! \brief Sets the # of nodes for which no successor could be generated
        inline void incrementNodesDeadend(unsigned int inc = 1);

        //! \brief Sets the # of nodes pruned
        inline void incrementNodesPruned(unsigned int inc = 1);

        //! \copydoc SearchStatisticsBase
        std::ostream& print(std::ostream& os) const override;

        //! \copydoc SearchStatisticsBase
        void serializeToJson(nlohmann::json& j) const override;

       private:
        unsigned int m_nodes_expanded;   //!< # of nodes for which successors were generated
        unsigned int m_nodes_evaluated;  //!< # of nodes for which heuristic was computed
        unsigned int m_nodes_generated;  //! # of nodes created in total
        unsigned int m_nodes_reopened;   //! # of *closed* nodes which were reopened
        unsigned int m_nodes_deadend;    //! # of nodes for which no successor could be generated
        unsigned int m_nodes_pruned;     //! # of nodes pruned
    };

    std::ostream& operator<<(std::ostream& os, const SearchStatisticsCommon& stats);
    // todo: json

    // Inline Functions
    unsigned int SearchStatisticsCommon::numberOfNodesExpanded() const
    {
        return m_nodes_expanded;
    }

    unsigned int SearchStatisticsCommon::numberOfNodesEvaluated() const
    {
        return m_nodes_evaluated;
    }

    unsigned int SearchStatisticsCommon::numberOfNodesGenerated() const
    {
        return m_nodes_generated;
    }

    unsigned int SearchStatisticsCommon::numberOfNodesReopened() const
    {
        return m_nodes_reopened;
    }

    unsigned int SearchStatisticsCommon::numberOfDeadendNodes() const
    {
        return m_nodes_deadend;
    }

    unsigned int SearchStatisticsCommon::numberOfNodesPruned() const
    {
        return m_nodes_pruned;
    }

    void SearchStatisticsCommon::incrementNodesExpanded(unsigned int inc)
    {
        m_nodes_expanded += inc;
    }

    void SearchStatisticsCommon::incrementNodesEvaluated(unsigned int inc)
    {
        m_nodes_evaluated += inc;
    }

    void SearchStatisticsCommon::incrementNodesGenerated(unsigned int inc)
    {
        m_nodes_generated += inc;
    }

    void SearchStatisticsCommon::incrementNodesReopened(unsigned int inc)
    {
        m_nodes_reopened += inc;
    }

    void SearchStatisticsCommon::incrementNodesDeadend(unsigned int inc)
    {
        m_nodes_deadend += inc;
    }

    void SearchStatisticsCommon::incrementNodesPruned(unsigned int inc)
    {
        m_nodes_pruned += inc;
    }
}  // namespace grstapse