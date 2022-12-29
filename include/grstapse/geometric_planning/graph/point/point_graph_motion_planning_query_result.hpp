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
#include "grstapse/geometric_planning/graph/graph_motion_planning_query_result.hpp"

namespace grstapse
{
    // Forward Declarations
    class PointGraphConfiguration;

    /**!
     * Motion planning query results of a graph search through an undirected graph where vertices represent points in
     * 2D space
     */
    class PointGraphMotionPlanningQueryResult : public GraphMotionPlanningQueryResult
    {
       public:
        //! Constructor
        PointGraphMotionPlanningQueryResult(MotionPlannerQueryStatus status,
                                            const std::vector<std::shared_ptr<PointGraphConfiguration>>& path =
                                                std::vector<std::shared_ptr<PointGraphConfiguration>>());

        [[nodiscard]] inline const std::vector<std::shared_ptr<PointGraphConfiguration>>& path() const;

        //! \copydoc MotionPlanningQueryResultBase
        [[nodiscard]] float length() const final override;

        //! \copydoc MotionPlanningQueryResultBase
        void serializeToJson(nlohmann::json& j) const final override;

       private:
        std::vector<std::shared_ptr<PointGraphConfiguration>> m_path;
    };

    // Inline Functions
    const std::vector<std::shared_ptr<PointGraphConfiguration>>& PointGraphMotionPlanningQueryResult::path() const
    {
        return m_path;
    }
}  // namespace grstapse