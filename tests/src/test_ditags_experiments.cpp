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

    int num_problem          = 50;
    std::string problem_name = "survivor_problem";
    std::string path         = "./data/Problems/WAFR/";
    std::string base_ext     = "Base/";
    std::string alt_ext      = "alts/alt_";
    std::string out_ext      = "outputs/";
    std::vector<std::string> alts{
        "duration_up",
        "duration_down",
        //"reqs_up",
        //"reqs_down",
        //"traits_up",
        //"traits_down",
        //"gain_task",
        //"gained_agent",
        //"lost_agent",
        //"lost_task"

    };

    int num_problem_comp      = 50;
    std::string path_comp     = "./data/Problems/WAFR/Compare";
    std::string base_ext_comp = "/Comp/";
    std::string out_ext_comp  = "/Output/";

    void runSingleExperiment(int i)
    {
        std::string base_file        = path + base_ext + problem_name + std::to_string(i) + ".json";
        std::string base_output_file = path + out_ext + problem_name + std::to_string(i) + ".json";

        std::cout << base_file << std::endl;
        std::cout << base_output_file << std::endl;

        std::ifstream fin(base_file);
        nlohmann::json k;
        fin >> k;
        fin.close();
        std::shared_ptr<const ItagsProblemInputs> problem_inputs = k.get<std::shared_ptr<ItagsProblemInputs>>();
        DitagsTetaq ditags(problem_inputs);
        SearchResults<DynIncrementalTaskAllocationNode> results = ditags.search();
        if(results.foundGoal())
        {
            ditags.writeSolutionToFile(base_output_file, results.goal());
        }

        TimeKeeper::instance().reset(ditags.m_parameters->timer_name);
        TimeKeeper::instance().reset(constants::k_scheduling_time);
        TimeKeeper::instance().reset(constants::k_motion_planning_time);

        if(results.foundGoal())
        {
            for(int j = 0; j < alts.size(); ++j)
            {
                std::string alt_file = path + alt_ext + alts[j] + "/" + problem_name + std::to_string(i) + ".json";
                std::cout << alt_file << std::endl;

                std::ifstream fin_repair(alt_file);
                nlohmann::json h;
                fin_repair >> h;
                fin_repair.close();
                std::shared_ptr<const ItagsProblemInputs> problem_inputs_alt =
                    h.get<std::shared_ptr<ItagsProblemInputs>>();

                TimeKeeper::instance().reset(ditags.m_parameters->timer_name);
                TimeKeeper::instance().reset(constants::k_scheduling_time);
                TimeKeeper::instance().reset(constants::k_motion_planning_time);

                std::cout << "Repair" << std::endl;
                DitagsTetaq ditags_repair = ditags.getDeepCopy();
                ditags_repair.repairSearch(problem_inputs_alt);
                SearchResults<DynIncrementalTaskAllocationNode> results_repair = ditags_repair.continueSearch();
                std::string repair_output_file =
                    path + out_ext + "repair_" + alts[j] + problem_name + std::to_string(i) + ".json";
                if(results_repair.foundGoal())
                {
                    ditags_repair.writeSolutionToFile(repair_output_file, results_repair.goal());
                }
                TimeKeeper::instance().reset(ditags_repair.m_parameters->timer_name);
                TimeKeeper::instance().reset(constants::k_scheduling_time);
                TimeKeeper::instance().reset(constants::k_motion_planning_time);

                if(results_repair.foundGoal())
                {
                    std::cout << "Replan" << std::endl;
                    std::string rerun_output_file =
                        path + out_ext + "rerun_" + alts[j] + problem_name + std::to_string(i) + ".json";
                    DitagsTetaq ditags_rerun(problem_inputs_alt);
                    SearchResults<DynIncrementalTaskAllocationNode> results_rerun = ditags_rerun.search();
                    if(results_rerun.foundGoal())
                    {
                        ditags_rerun.writeSolutionToFile(rerun_output_file, results_rerun.goal());
                    }
                }

                TimeKeeper::instance().reset(ditags.m_parameters->timer_name);
                TimeKeeper::instance().reset(constants::k_scheduling_time);
                TimeKeeper::instance().reset(constants::k_motion_planning_time);
            }
        }
        else
        {
            std::cout << "Failed";
        }
    }

    TEST(Ditags_exp, Experiments)
    {
        for(int i = 0; i < num_problem; ++i)
        {
            runSingleExperiment(i);
        }
        return;
    }

    void runSingleCompExperiment(int i)
    {
        std::string base_file        = path_comp + base_ext_comp + problem_name + std::to_string(i) + ".json";
        std::string base_output_file = path_comp + out_ext_comp + "out_" + problem_name + std::to_string(i) + ".json";

        std::cout << base_file << std::endl;
        std::cout << base_output_file << std::endl;

        std::ifstream fin(base_file);
        nlohmann::json k;
        fin >> k;
        fin.close();
        std::cout << k << std::endl;
        std::shared_ptr<const ItagsProblemInputs> problem_inputs = k.get<std::shared_ptr<ItagsProblemInputs>>();
        std::cout << problem_inputs << std::endl;
        DitagsTetaq ditags(problem_inputs);
        SearchResults<DynIncrementalTaskAllocationNode> results = ditags.search();
        if(results.foundGoal())
        {
            std::cout << "Solved" << std::endl;
            ditags.writeSolutionToFile(base_output_file, results.goal());
        }

        TimeKeeper::instance().reset(ditags.m_parameters->timer_name);
        TimeKeeper::instance().reset(constants::k_scheduling_time);
        TimeKeeper::instance().reset(constants::k_motion_planning_time);
    }

    TEST(Ditags_exp, ExperimentsComp)
    {
        for(int i = 24; i < 25; ++i)
        {
            runSingleCompExperiment(i);
        }
        return;
    }

}  // namespace grstapse::unittests

// 4 17 18