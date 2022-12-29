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
#include <tuple>
#include <vector>
// Local
#include "grstapse/scheduling/schedule_base.hpp"

namespace grstapse {
    //! \brief Container for a deterministic schedule for a set of tasks with constraints
    class DeterministicSchedule : public ScheduleBase {
    public:
        //! \brief Default Constructor
        DeterministicSchedule() = default;

        //! \brief Copy Constructor
        DeterministicSchedule(DeterministicSchedule const &);

        /**!
         * \brief Full Constructor
         *
         * \param makespan The total execution time for the schedule
         * \param timepoints A list of the start/end timepoints for a set of tasks
         * \param precedence_set_mutex_constraints A list of the precedence set mutex constraints
         */
        DeterministicSchedule(
                const float makespan,
                const std::vector<std::pair<float, float>> &timepoints,
                const std::vector<std::pair<unsigned int, unsigned int>> &precedence_set_mutex_constraints);

        /**!
         * \returns The list of time points where timepoints()[i].first is the start of the ith task/action and
         *          timepoints()[i].second is the end of the ith task/action
         */
        [[nodiscard]] inline const std::vector<std::pair<float, float>> &timepoints() const;

        //! \returns When the ith task starts
        [[nodiscard]] inline float taskStart(const unsigned int i) const;

        //! \returns When the ith task ends
        [[nodiscard]] inline float taskEnd(const unsigned int i) const;

    protected:
        std::vector<std::pair<float, float>> m_timepoints;
    };

    // Inline Functions
    const std::vector<std::pair<float, float>> &DeterministicSchedule::timepoints() const {
        return m_timepoints;
    }

    float DeterministicSchedule::taskStart(const unsigned int i) const {
        return m_timepoints[i].first;
    }

    float DeterministicSchedule::taskEnd(const unsigned int i) const {
        return m_timepoints[i].second;
    }

}  // namespace grstapse