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
#include <cassert>
#include <string>
#include <vector>

namespace grstapse
{
    /**!
     * Load and save pgm files
     */
    class Pgm
    {
       public:
        //! Default Constructor
        Pgm() = default;

        //! Constructor
        explicit Pgm(const std::string& filepath);

        //! Load the image from a .pgm file
        void loadFile(const std::string& filepath);

        //! Saves the image to a .pgm file
        void saveFile(const std::string& filepath);

        //! \returns The width of the loaded image
        [[nodiscard]] inline unsigned int width() const
        {
            return m_width;
        }

        //! \returns The height of the loaded image
        [[nodiscard]] inline unsigned int height() const
        {
            return m_height;
        }

        const std::vector<unsigned int>& pixels() const
        {
            return m_pixels;
        }

        [[nodiscard]] inline unsigned int pixel(const unsigned int row, const unsigned int column) const
        {
            assert(row < m_height);
            assert(column < m_width);
            return m_pixels[row * m_width + column];
        }

       private:
        std::vector<unsigned int> m_pixels;
        unsigned int m_width;
        unsigned int m_height;
    };

}  // namespace grstapse