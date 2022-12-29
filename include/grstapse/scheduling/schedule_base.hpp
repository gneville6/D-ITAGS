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

namespace grstapse {
    /**!
     * Base class for a container that represents information about a schedule
     */
    class ScheduleBase {
    public:
        //! Default Constructor
        ScheduleBase();

        //! virtual destructor to make this class polymorphic
        virtual ~ScheduleBase() = default;

        //! Constructor
        ScheduleBase(float makespan,
                     const std::vector<std::pair<unsigned int, unsigned int>> &precedence_set_mutex_constraints);

        //! Copy Constructor
        ScheduleBase(ScheduleBase const &);

        //! \returns The makespan (or the total execution time) of the schedule
        [[nodiscard]] inline float makespan() const;

        //! \returns A list of the precedence set mutex constraints
        [[nodiscard]] inline const std::vector<std::pair<unsigned int, unsigned int>> &precedenceSetMutexConstraints()
        const;

    protected:
        float m_makespan;
        std::vector<std::pair<unsigned int, unsigned int>> m_precedence_set_mutex_constraints;
    };

    // Inline Functions
    float ScheduleBase::makespan() const {
        return m_makespan;
    }

    const std::vector<std::pair<unsigned int, unsigned int>> &ScheduleBase::precedenceSetMutexConstraints() const {
        return m_precedence_set_mutex_constraints;
    }

}  // namespace grstapse