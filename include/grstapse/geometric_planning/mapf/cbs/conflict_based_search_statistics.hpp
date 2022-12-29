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
#include "grstapse/common/search/search_statistics_base.hpp"

namespace grstapse
{
    /**!
     * Statistics for a Conflict-Based Search
     *
     * \see ConflictBasedSearch
     */
    class ConflictBasedSearchStatistics : public SearchStatisticsBase
    {
       public:
        ConflictBasedSearchStatistics() = default;

        //! \returns The number of high level nodes generated
        [[nodiscard]] inline unsigned int numberOfHighLevelNodesGenerated() const noexcept;

        //! \brief Increments the record of the number of high level nodes generated
        inline void incrementNumberOfHighLevelNodesGenerated(unsigned int inc = 1) noexcept;

        //! \returns The number of high level nodes evaluated
        [[nodiscard]] inline unsigned int numberOfHighLevelNodesEvaluated() const noexcept;

        //! \brief Increments the record of the number of high level nodes evaluated
        inline void incrementNumberOfHighLevelNodesEvaluated(unsigned int inc = 1) noexcept;

        //! \returns The number of high level nodes expanded
        [[nodiscard]] inline unsigned int numberOfHighLevelNodesExpanded() const noexcept;

        //! \brief Increments the record of the number of high level nodes expanded
        inline void incrementNumberOfHighLevelNodesExpanded(unsigned int inc = 1) noexcept;

        //! \returns The time spent on the high level search (excludes the time spent on the low level search)
        [[nodiscard]] inline float highLevelTime() const noexcept;

        /**!
         * \returns The total time spent on the search (includes both the time for the high level search and all the
         *          time for the low level searches)
         */
        [[nodiscard]] inline float totalTime() const noexcept;

        //! \brief Sets the amount of time for the whole search (includes the time for the low level searches)
        inline void setTotalTime(float time) noexcept;

        //! \returns The total number of nodes in the low level searches that were generated
        [[nodiscard]] inline unsigned int numberOflowLevelNodesGenerated() const noexcept;

        //! \brief Increments the record of the total number of nodes in the low level searches that were generated
        inline void incrementNumberOfLowLevelNodesGenerated(unsigned int inc = 1) noexcept;

        //! \returns The total number of nodes in the low level searches that were expanded
        [[nodiscard]] inline unsigned int numberOfLowLevelNodesEvaluated() const noexcept;

        //! \brief Increments the record of the total number of nodes in the low level searches that were evaluated
        inline void incrementNumberOfLowLevelNodesEvaluated(unsigned int inc = 1) noexcept;

        //! \returns The total number of nodes in the low level searches that were expanded
        [[nodiscard]] inline unsigned int numberOfLowLevelNodesExpanded() const noexcept;

        //! \brief Increments the record of the total number of nodes in the low level searches that were expanded
        inline void incrementNumberOfLowLevelNodesExpanded(unsigned int inc = 1) noexcept;

        //! \returns The total number of nodes in the low level searches that were deadends
        [[nodiscard]] inline unsigned int numberOfLowLevelDeadendNodes() const noexcept;

        //! \brief Increments the record of the total number of nodes in the low level searches that were deadends
        inline void incrementNumberOfLowLevelDeadendNodes(unsigned int inc = 1) noexcept;

        //! \returns The total number of nodes pruned during the low level searches
        [[nodiscard]] inline unsigned int numberOfLowLevelNodesPruned() const noexcept;

        //! \brief Increments the record of the total number of nodes in the low level searches that were pruned
        inline void incrementNumberOfLowLevelNodesPruned(unsigned int inc = 1) noexcept;

        //! \returns The total time spent on the low level searches
        [[nodiscard]] inline float lowLevelTime() const noexcept;

        //! \brief Adds to the record of the time spent on low level searches
        inline void addLowLevelTime(float inc) noexcept;

        //! \copydoc SearchStatisticsBase
        std::ostream& print(std::ostream& os) const override;

        //! \copydoc SearchStatisticsBase
        void serializeToJson(nlohmann::json& j) const override;

       private:
        unsigned int m_high_level_nodes_generated;
        unsigned int m_high_level_nodes_evaluated;
        unsigned int m_high_level_nodes_expanded;
        float m_total_time;

        unsigned int m_low_level_nodes_generated;
        unsigned int m_low_level_nodes_evaluated;
        unsigned int m_low_level_nodes_expanded;
        unsigned int m_low_level_nodes_deadends;
        unsigned int m_low_level_nodes_pruned;
        float m_low_level_time;
    };

    // Inline functions
    unsigned int ConflictBasedSearchStatistics::numberOfHighLevelNodesGenerated() const noexcept
    {
        return m_high_level_nodes_generated;
    }

    void ConflictBasedSearchStatistics::incrementNumberOfHighLevelNodesGenerated(unsigned int inc) noexcept
    {
        m_high_level_nodes_generated += inc;
    }

    unsigned int ConflictBasedSearchStatistics::numberOfHighLevelNodesEvaluated() const noexcept
    {
        return m_high_level_nodes_evaluated;
    }

    void ConflictBasedSearchStatistics::incrementNumberOfHighLevelNodesEvaluated(unsigned int inc) noexcept
    {
        m_high_level_nodes_evaluated += 1;
    }

    unsigned int ConflictBasedSearchStatistics::numberOfHighLevelNodesExpanded() const noexcept
    {
        return m_high_level_nodes_expanded;
    }

    void ConflictBasedSearchStatistics::incrementNumberOfHighLevelNodesExpanded(unsigned int inc) noexcept
    {
        m_high_level_nodes_expanded += inc;
    }

    float ConflictBasedSearchStatistics::highLevelTime() const noexcept
    {
        return m_total_time - m_low_level_time;
    }

    float ConflictBasedSearchStatistics::totalTime() const noexcept
    {
        return m_total_time;
    }

    void ConflictBasedSearchStatistics::setTotalTime(float time) noexcept
    {
        m_total_time = time;
    }

    unsigned int ConflictBasedSearchStatistics::numberOflowLevelNodesGenerated() const noexcept
    {
        return m_low_level_nodes_generated;
    }

    void ConflictBasedSearchStatistics::incrementNumberOfLowLevelNodesGenerated(unsigned int inc) noexcept
    {
        m_low_level_nodes_generated += inc;
    }

    unsigned int ConflictBasedSearchStatistics::numberOfLowLevelNodesEvaluated() const noexcept
    {
        return m_low_level_nodes_evaluated;
    }

    void ConflictBasedSearchStatistics::incrementNumberOfLowLevelNodesEvaluated(unsigned int inc) noexcept
    {
        m_low_level_nodes_evaluated += inc;
    }
    unsigned int ConflictBasedSearchStatistics::numberOfLowLevelNodesExpanded() const noexcept
    {
        return m_low_level_nodes_expanded;
    }

    void ConflictBasedSearchStatistics::incrementNumberOfLowLevelNodesExpanded(unsigned int inc) noexcept
    {
        m_low_level_nodes_expanded += inc;
    }

    unsigned int ConflictBasedSearchStatistics::numberOfLowLevelDeadendNodes() const noexcept
    {
        return m_low_level_nodes_deadends;
    }

    void ConflictBasedSearchStatistics::incrementNumberOfLowLevelDeadendNodes(unsigned int inc) noexcept
    {
        m_low_level_nodes_deadends += inc;
    }

    unsigned int ConflictBasedSearchStatistics::numberOfLowLevelNodesPruned() const noexcept
    {
        return m_low_level_nodes_pruned;
    }

    void ConflictBasedSearchStatistics::incrementNumberOfLowLevelNodesPruned(unsigned int inc) noexcept
    {
        m_low_level_nodes_pruned += inc;
    }

    float ConflictBasedSearchStatistics::lowLevelTime() const noexcept
    {
        return m_low_level_time;
    }

    void ConflictBasedSearchStatistics::addLowLevelTime(float inc) noexcept
    {
        m_low_level_time += inc;
    }
}  // namespace grstapse