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
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>

// External
#include <gtest/gtest.h>
#include <nlohmann/json.hpp>
// Project
#include <grstapse/common/utilities/custom_json_conversions.hpp>
#include <grstapse/task_allocation/itags/ditags_tetaq.hpp>
#include <grstapse/task_allocation/itags/itags_problem_inputs.hpp>
#include <grstapse/task_allocation/itags/robot_traits_matrix_reduction.hpp>
#include <grstapse/task_allocation/itags/time_extended_task_allocation_quality.hpp>

namespace grstapse::unittests
{
    // 0,  2,  4,  5,  6,  8,  9,  11, 12, 13, 14, 15, 19, 20, 21, 22, 29, 31, 32, 35, 45
    // New Run
    // 3,  4,  5,  6,  8,  9,  12, 16, 18, 19, 21, 22, 23, 24, 25, 26, 28, 29, 33, 46, 55, 48, 49, 59, 60, 62, 64,
    // 65, 71, 75, 77, 78, 87, 89, 92, 94, 99 fix 25, 19
    // apr
    int num_problem_proof[40] = {0,  2,  6,  8,  9,  11, 12, 13, 15, 16, 17, 18, 20, 23, 24, 26, 27, 28,
                                 30, 32, 33, 34, 36, 38, 40, 41, 42, 44, 45, 46, 47, 49, 50, 51, 52, 53};
    // could rerun 35
    float alphas[25] = {
        1.00, 0.97, 0.95, 0.93, 0.91, 0.89, 0.87, 0.85, 0.83, 0.81, 0.79, 0.77, 0.75,
        0.73, 0.71, 0.69, 0.67, 0.65, 0.63, 0.61, 0.59, 0.57, 0.55, 0.53, 0.51};  //, 0.40, 0.30, 0.20,
                                                                                  // 0.10, 0.00};  // 20;
    // float alphas[2]                = {1.00, 0.00};  // 20;
    std::string problem_name_proof = "survivor_problem";
    std::string path_proof         = "./data/Problems/";
    std::string base_ext_proof     = "Very_large/";
    std::string out_ext_proof      = "WAFR_Proof/outputs_large/alpha_";

    void runSingleExperiment_proof(int prob_num)
    {
        std::string base_file = path_proof + base_ext_proof + problem_name_proof + std::to_string(prob_num) + ".json";
        std::string base_output_file = path_proof + out_ext_proof + std::to_string(alphas[0]) + problem_name_proof +
                                       std::to_string(prob_num) + ".json";

        std::cout << base_file << std::endl;
        std::cout << base_output_file << std::endl;

        std::ifstream fin(base_file);
        nlohmann::json k;
        fin >> k;
        fin.close();
        std::shared_ptr<ItagsProblemInputs> problem_inputs = k.get<std::shared_ptr<ItagsProblemInputs>>();
        DitagsTetaq ditags(problem_inputs);
        SearchResults<DynIncrementalTaskAllocationNode> results = ditags.search();
        if(results.foundGoal())
        {
            ditags.writeSolutionToFile(base_output_file, results.goal());
        }
        float worst_makespan = results.goal()->m_schedule->makespan();
        for(float i: alphas)
        {
            std::string base_output_file = path_proof + out_ext_proof + std::to_string(i) + problem_name_proof +
                                           std::to_string(prob_num) + ".json";
            std::cout << "Alpha=" << i << std::endl;
            problem_inputs->m_schedule_worst_makespan = worst_makespan;
            problem_inputs->m_alpha                   = i;
            DitagsTetaq ditags(problem_inputs);
            SearchResults<DynIncrementalTaskAllocationNode> results = ditags.search();
            if(results.foundGoal())
            {
                ditags.writeSolutionToFile(base_output_file, results.goal());
            }
        }
    }

    TEST(Ditags_exp_proof, Experiments)
    {
        for(const int i: num_problem_proof)  //(int i = 0; i < num_problem_proof; ++i)
        {
            runSingleExperiment_proof(i);
        }
        return;
    }

}  // namespace grstapse::unittests
