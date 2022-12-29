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
#include "grstapse/common/utilities/error.hpp"

// External
#include <fmt/format.h>

// Local
#include "grstapse/common/utilities/logger.hpp"

namespace grstapse
{
    std::logic_error createLogicError(const std::string& formatted_message,
                                      const std::experimental::source_location location)
    {
        const std::string error_message = fmt::format("<logic_error> in {0:s} at {1:s}:{2:d}) {3:s}",
                                                      location.function_name(),
                                                      location.file_name(),
                                                      location.line(),
                                                      formatted_message);
        Logger::error(error_message);
#if NDEBUG
        exit(1);
#else
        return std::logic_error(error_message);
#endif
    }

    std::runtime_error createRuntimeError(const std::string& formatted_message,
                                          const std::experimental::source_location location)
    {
        const std::string error_message = fmt::format("<runtime_error> in {0:s} at {1:s}:{2:d}) {3:s}",
                                                      location.function_name(),
                                                      location.file_name(),
                                                      location.line(),
                                                      formatted_message);
        Logger::error(error_message);
#if NDEBUG
        exit(1);
#else
        return std::runtime_error(error_message);
#endif
    }
}  // namespace grstapse