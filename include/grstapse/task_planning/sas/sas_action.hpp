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
#include <string>
// External
// Local

namespace grstapse
{
    // Forward Declarations

    /**!
     * Container for a SAS+ Action
     *
     * \note SAS+ Actions are always grounded as there are no parameters in the formalism
     *
     * \todo(Andrew): Cite SAS+ Formalism
     * \cite
     */
    class SasAction
    {
       public:
        //! Constructor
        SasAction(const std::string& name, const float duration);

        //! \returns The name of this action
        [[nodiscard]] inline const std::string& name() const;

        //! \returns The duration of this action
        [[nodiscard]] inline float duration() const;

       private:
        unsigned int m_id;
        std::string m_name;
        float m_duration;

        static unsigned int s_next_id;
    };

    // Inline functions
    const std::string& SasAction::name() const
    {
        return m_name;
    }
    float SasAction::duration() const
    {
        return m_duration;
    }

}  // namespace grstapse