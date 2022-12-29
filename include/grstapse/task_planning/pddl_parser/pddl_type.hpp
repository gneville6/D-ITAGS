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
#include <string>

// Local
#include "grstapse/task_planning/pddl_parser/pddl_base.hpp"

namespace grstapse
{
    //! Container for a PDDL type
    class PddlType : public PddlBase
    {
       public:
        /**!
         * Constructor
         *
         * \param name The name of this type
         * \param parent A pointer to the parent of this type
         */
        PddlType(const std::string& name, std::shared_ptr<const PddlType> parent);

        //! \returns The name of this type
        [[nodiscard]] const std::string& name() const;

        //! \returns The identifier for this type
        [[nodiscard]] unsigned int id() const;

        //! \returns A pointer to the parent of this type
        [[nodiscard]] std::shared_ptr<const PddlType> parent() const;

        /**!
         * \brief Prints a PDDL representation to the given stream
         *
         * \param os The stream to print to
         * \returns \p os
         */
        std::ostream& print(std::ostream& o) const override;

       private:
        std::string m_name;
        std::shared_ptr<const PddlType> m_parent;
        unsigned int m_id;

        static unsigned int s_next_id;
    };
}  // namespace grstapse