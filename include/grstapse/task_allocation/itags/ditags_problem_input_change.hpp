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

// Global
#include <memory>
#include <set>
#include <tuple>
#include <vector>
// External
#include <Eigen/Core>
// Local
#include "grstapse/grstaps_problem_inputs.hpp"

namespace grstapse
{
    // Forward Declarations
    class Task;

    /**!
     * Contain for the inputs to an ITAGS problem
     */
    class DtagsProblemInputsChanges
    {
       public:
        /**!
         * Default Constructor for changes object
         */
        DtagsProblemInputsChanges();

        /**!
         * Constructor for changes object
         */
        DtagsProblemInputsChanges(std::shared_ptr<const ItagsProblemInputs> problem_inputs,
                                  bool repair_open,
                                  bool repair_closed,
                                  bool repair_pruned,
                                  bool mp_changed,
                                  bool lost_agent,
                                  bool new_agent);

        /**!
         * Getter for problem inputs
         */
        std::shared_ptr<const ItagsProblemInputs> getNewInputs() const;
        /**!
         * Setter for problem inputs
         *
         * \param the new problem inputs to be set
         */
        void setNewInputs(std::shared_ptr<const ItagsProblemInputs> new_inputs);

        bool getMotionPlanChanged() const;
        /**!
         * Setter for if motion plan have changed bool
         *
         * \param bool for if mp has changed
         */
        void setMotionPlanChanged(bool mp_change);

        /**!
         * Getter for if motion plan have changed bool
         *
         */
        bool getNeedUpdateClosed() const;
        /**!
         * Setter for if the closed set needs updating
         *
         * \param bool for if closed needs updates
         */
        void setNeedUpdateClosed(bool repair_closed_needed);

        /**!
         * Getter for if the pruned set needs updating
         *
         */
        bool getNeedUpdatePruned() const;
        /**!
         * Setter for if the pruned set needs updating
         *
         * \param bool for if pruned needs updates
         */
        void setNeedUpdatePruned(bool repair_pruned_needed);

        /**!
         * Getter for if the open set needs updating
         *
         */
        bool getNeedUpdateOpen() const;
        /**!
         * Setter for if the open set needs updating
         *
         * \param bool for if open needs updates
         */
        void setNeedUpdateOpen(bool repair_open_needed);

        /**!
         * Getter for if an agent was lost
         *
         */
        bool getLostAgent() const;
        /**!
         * Setter for if an agent was lost
         *
         * \param bool for if agent was lost
         */
        void setLostAgent(bool lost_agent);

        /**!
         * Getter for if an agent was gained
         *
         */
        bool getNewAgent() const;
        /**!
         * Setter for if an agent was gained
         *
         * \param bool for if agent was gained
         */
        void setNewAgent(bool new_agent);

       protected:
        // From task planning
        std::shared_ptr<const ItagsProblemInputs> m_new_inputs;

        // Bools used for Repair
        bool m_repair_closed_needed;
        bool m_repair_pruned_needed;
        bool m_repair_open_needed;
        bool m_mp_changed;
        bool m_new_agent;
        bool m_lost_agent;
    };
}  // namespace grstapse

// uncovered
// change in number of traits