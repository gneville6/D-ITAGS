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
// Global
#include <fstream>
#include <memory>
// External
#include <gtest/gtest.h>
#include <nlohmann/json.hpp>
// Project
#include <grstapse/common/utilities/custom_json_conversions.hpp>
#include <grstapse/task_allocation/itags/itags.hpp>
#include <grstapse/task_allocation/itags/itags_problem_inputs.hpp>
#include <grstapse/task_allocation/itags/time_extended_task_allocation_quality.hpp>

namespace grstapse::unittests {
#ifndef NO_MILP
    /**!
     * TODO(Andrew,Glen): better tests
     */
    TEST(Itags, Simple) {
        std::ifstream fin("data/task_allocation/itags_problem_inputs/full_run.json");
        nlohmann::json j;
        fin >> j;
        std::shared_ptr<const ItagsProblemInputs> problem_inputs = j.get<std::shared_ptr<ItagsProblemInputs>>();
        Itags itags(problem_inputs);

        SearchResults<IncrementalTaskAllocationNode, SearchStatisticsCommon> results = itags.search();
        std::shared_ptr<SearchStatisticsCommon> statistics = results.statistics();
    }

    /**!
     * TODO(Andrew,Glen): better tests
     */
    TEST(Itags, Hard) {}

    /**!
     * TODO(Andrew,Glen): better tests
     */
    TEST(Itags, WriteSolution) {
        std::ifstream fin("data/task_allocation/itags_problem_inputs/full_run.json");
        nlohmann::json j;
        fin >> j;
        std::shared_ptr<const ItagsProblemInputs> problem_inputs = j.get<std::shared_ptr<ItagsProblemInputs>>();
        Itags<TimeExtendedTaskAllocationQuality<IncrementalTaskAllocationNode>> itags(problem_inputs);

        SearchResults<IncrementalTaskAllocationNode, SearchStatisticsCommon> results = itags.search();
        itags.writeSolutionToFile("itags_test_output.json", results.goal());
    }

    TEST(Itags, GlenDitagsTest) {
        std::ifstream in("data/task_allocation/itags_problem_inputs/survivor_problem0.json");
        nlohmann::json j;
        in >> j;
        auto itags_problem_inputs = j.get<std::shared_ptr<ItagsProblemInputs>>();
        Itags<TimeExtendedTaskAllocationQuality<IncrementalTaskAllocationNode>, IncrementalTaskAllocationNode> itags(
                itags_problem_inputs);
        auto result = itags.search();
        float mp_time = TimeKeeper::instance().time(constants::k_motion_planning_time);
        float smp_time = TimeKeeper::instance().time(constants::k_scheduling_time);
        float s_time = smp_time - mp_time;
        float total_time = TimeKeeper::instance().time("itags");
        float ta_time = total_time - smp_time;
        std::cout << fmt::format(
                "MP Time: {0:f}s\nS Time: {1:f}s\nTA Time: {2:f}\nTotal Time: {3:f}\nNum MILPs run: {4:d}\n",
                mp_time,
                s_time,
                ta_time,
                total_time,
                MilpSchedulerBase::numIterations());
    }

#endif
}  // namespace grstapse::unittests