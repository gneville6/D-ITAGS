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
#include <memory>
// External
#include <nlohmann/json.hpp>
// Local
#include "grstapse/common/utilities/timer.hpp"

namespace grstapse
{
    // Forward Declarations
    class SchedulerProblemInputs;
    class SchedulerParameters;
    class ScheduleBase;

    //! \brief Abstract base class for a scheduling algorithm
    class SchedulerBase
    {
       public:
        /**!
         * \brief Constructor
         *
         * \param problem_inputs The inputs for a scheduling problem
         * \param parameters The parameters for configuring the scheduling algorithm
         */
        explicit SchedulerBase(const std::shared_ptr<const SchedulerProblemInputs>& problem_inputs);

        /**!
         * \brief Solves the scheduling problem
         *
         * \returns The solution to the problem
         */
        std::shared_ptr<const ScheduleBase> solve();

        //! \returns The number of time that scheduling has failed
        [[nodiscard]] static unsigned int numFailures();

       protected:
        /**!
         * \brief Solves the scheduling problem
         *
         * \returns The solution to the problem
         */
        virtual std::shared_ptr<const ScheduleBase> computeSchedule() = 0;

        std::shared_ptr<const SchedulerProblemInputs> m_problem_inputs;

        static unsigned int s_num_failures;
    };
}  // namespace grstapse