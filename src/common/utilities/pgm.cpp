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
#include "grstapse/common/utilities/pgm.hpp"

// Global
#include <exception>
#include <fstream>
#include <sstream>

namespace grstapse
{
    Pgm::Pgm(const std::string& filepath)
    {
        loadFile(filepath);
    }

    void Pgm::loadFile(const std::string& filepath)
    {
        std::ifstream fin(filepath.c_str());

        std::string file_type;
        getline(fin, file_type);
        for(int i = 0; i < file_type.size(); ++i)
        {
            if(file_type[i] == ' ')
            {
                continue;
            }
            // Comment
            else if(file_type[i] == '#')
            {
                // Grab the next line and start over
                getline(fin, file_type);
                i = -1;
                continue;
            }
            // Not comment
            else
            {
                break;
            }
        }

        if(file_type != "P5" && file_type != "P2")
        {
            throw std::invalid_argument("Invalid PGM image type: " + file_type);
        }

        std::string dimensions;
        getline(fin, dimensions);
        for(int i = 0; i < dimensions.size(); ++i)
        {
            if(dimensions[i] == ' ')
            {
                continue;
            }
            // Comment
            else if(dimensions[i] == '#')
            {
                // Grab the next line and start over
                getline(fin, dimensions);
                i = -1;
                continue;
            }
            // Not comment
            else
            {
                break;
            }
        }

        std::istringstream iss(dimensions);
        iss >> m_width >> m_height;

        m_pixels.resize(m_width * m_height);

        std::string max_val_str;
        getline(fin, max_val_str);

        unsigned int i = 0;
        if(file_type == "P2")
        {
            while(fin.good())
            {
                std::string line;
                getline(fin, line);

                std::istringstream iss2(line);
                std::string val;
                while(iss2 >> val)
                {
                    m_pixels[i++] = atoi(val.c_str());
                }
            }
        }
        else  // file_type == "P5"
        {
            while(fin.good())
            {
                char lo;
                fin.get(lo);
                m_pixels[i++] = static_cast<unsigned int>(lo);
            }
        }

        fin.close();
    }

    void Pgm::saveFile(const std::string& filepath)
    {
        // TODO(andrew)
        throw std::logic_error("Not implemented");
    }
}  // namespace grstapse