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
// Global
#include <fstream>
#include <memory>
// External
#include <gtest/gtest.h>
#include <nlohmann/json.hpp>
// Project
#include <filesystem>

#include <grstapse/common/utilities/custom_json_conversions.hpp>
#include <grstapse/task_allocation/itags/itags.hpp>
#include <grstapse/task_allocation/itags/itags_problem_inputs.hpp>
#include <grstapse/task_allocation/itags/itags_v1_to_v2_problem_converter.hpp>
#include <grstapse/task_allocation/itags/time_extended_task_allocation_quality.hpp>

namespace grstapse {
    TEST(ProblemTranslator, Simple) {
        //ItagsV1ToV2ProblemConverter problemConverter;
        //problemConverter.convertProblem("/home/debugger/cmake-build-debug/tests/data/problem_converter/config.json");
    }
}  // namespace grstapse