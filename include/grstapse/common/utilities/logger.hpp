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
#include <filesystem>
#include <memory>
#include <utility>
#include <vector>

// External
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

// Local
#include "grstapse/config.hpp"

namespace grstapse
{
    /**!
     * \brief Wrapper for spdlog that sets up a logger that writes
     *        both to the console and a file
     */
    class Logger
    {
       public:
        /**!
         * Writes a message to the logger using fmt format with
         * the debug level
         *
         * \param message The format string for the log message
         * \param args The arguments to be format into the log message
         *
         * \tparam Args A list of argument types to formatted into the
         *              log message
         */
        template <typename... Args>
        static void debug(const std::string &message, Args &&...args)
        {
            getInstance()->debug(message.c_str(), std::forward<Args>(args)...);
        }

        /**!
         * Writes a message to the logger using fmt format with
         * the info level
         *
         * \param message The format string for the log message
         * \param args The arguments to be format into the log message
         *
         * \tparam Args A list of argument types to formatted into the
         *              log message
         */
        template <typename... Args>
        static void info(const std::string &message, Args &&...args)
        {
            getInstance()->info(message.c_str(), std::forward<Args>(args)...);
        }

        /**!
         * Writes a message to the logger using fmt format with
         * the warn level
         *
         * \param message The format string for the log message
         * \param args The arguments to be format into the log message
         *
         * \tparam Args A list of argument types to formatted into the
         *              log message
         */
        template <typename... Args>
        static void warn(const std::string &message, Args &&...args)
        {
            getInstance()->warn(message.c_str(), std::forward<Args>(args)...);
        }

        /**!
         * Writes a message to the logger using fmt format with
         * the error level
         *
         * \param message The format string for the log message
         * \param args The arguments to be format into the log message
         *
         * \tparam Args A list of argument types to formatted into the
         *              log message
         */
        template <typename... Args>
        static void error(const std::string &message, Args &&...args)
        {
            getInstance()->error(message.c_str(), std::forward<Args>(args)...);
        }

        /**!
         * Writes a message to the logger using fmt format with
         * the critical level
         *
         * \param message The format string for the log message
         * \param args The arguments to be format into the log message
         *
         * \tparam Args A list of argument types to formatted into the
         *              log message
         */
        template <typename... Args>
        static void critical(const std::string &message, Args &&...args)
        {
            getInstance()->critical(message.c_str(), std::forward<Args>(args)...);
        }

       private:
        //! \returns the logger as a singleton
        static std::shared_ptr<spdlog::logger> getInstance()
        {
            // Only initialize once
            static bool first = true;
            if(first)
            {
                first      = false;
                time_t now = time(nullptr);

                // Create the logs folder if it does not exist
                std::filesystem::path logs_folder = "logs";
                if(!std::filesystem::exists(logs_folder))
                {
                    std::filesystem::create_directory(logs_folder);
                }

                // Create logger sinks
                std::vector<spdlog::sink_ptr> sinks;
                sinks.push_back(std::make_shared<spdlog::sinks::stdout_color_sink_mt>());
                sinks.push_back(std::make_shared<spdlog::sinks::basic_file_sink_mt>(
                    fmt::format("logs/{}.txt", asctime(localtime(&now)))));

                // Configure a logger
                auto logger = std::make_shared<spdlog::logger>(PROJECT_NAME_STR, begin(sinks), end(sinks));
                spdlog::set_pattern("[%H:%M:%S %z] [thread %t] [%I] %v");
                spdlog::register_logger(logger);
                spdlog::set_level(spdlog::level::debug);
            }

            return spdlog::get(PROJECT_NAME_STR);
        }

        //! \brief Hides the constructor from the user
        Logger() = default;
    };
}  // namespace grstapse