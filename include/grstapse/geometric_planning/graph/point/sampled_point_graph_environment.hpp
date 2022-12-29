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
#include "grstapse/geometric_planning/graph/graph_environment.hpp"

namespace grstapse
{
    // Forward Declarations
    class PointGraphEnvironment;

    /**!
     *
     */
    class SampledPointGraphEnvironment : public GraphEnvironment
    {
       public:
        //! Constructor
        SampledPointGraphEnvironment();

        //! Adds one of the sampled graphs
        inline void addGraph(const std::shared_ptr<PointGraphEnvironment>& g);

        //! \returns The \p index'th graph environment
        [[nodiscard]] const std::shared_ptr<PointGraphEnvironment>& graph(unsigned int index) const;

        //! \returns The number of sampled graphs
        [[nodiscard]] inline unsigned int numGraphs() const;

        //! \copydoc EnvironmentBase
        [[nodiscard]] float longestPath() const override;

       private:
        std::vector<std::shared_ptr<PointGraphEnvironment>> m_graphs;

        friend class PointGraphEnvironment;
    };

    //! Deserialize from json
    void from_json(const nlohmann::json& j, SampledPointGraphEnvironment& environment);

    // Inline functions
    void SampledPointGraphEnvironment::addGraph(const std::shared_ptr<PointGraphEnvironment>& g)
    {
        m_graphs.push_back(g);
    }

    unsigned int SampledPointGraphEnvironment::numGraphs() const
    {
        return m_graphs.size();
    }

}  // namespace grstapse