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
#include <filesystem>
#include <fstream>
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

// Mock
#include "mock_ditags.hpp"

namespace grstapse::unittests
{
    TEST(DItags, SimpleSearch)
    {
        std::ifstream fin("./data/task_allocation/itags_problem_inputs/full_run.json");
        nlohmann::json j;
        fin >> j;
        std::shared_ptr<const ItagsProblemInputs> problem_inputs = j.get<std::shared_ptr<ItagsProblemInputs>>();
        mocks::MockDitags ditags(problem_inputs);

        SearchResults<DynIncrementalTaskAllocationNode> results = ditags.search();
        std::shared_ptr<SearchStatisticsCommon> statistics      = results.statistics();
    }

    TEST(DItags, TestGetters)
    {
        std::ifstream fin("./data/task_allocation/itags_problem_inputs/full_run.json");
        nlohmann::json j;
        fin >> j;
        std::shared_ptr<const ItagsProblemInputs> problem_inputs = j.get<std::shared_ptr<ItagsProblemInputs>>();
        mocks::MockDitags ditags(problem_inputs);

        // Test Getters and Setters for Closed Stale NSQ
        // First test the default then test setting both ways
        ASSERT_FALSE(ditags.getClosedStaleNSQ());
        ditags.setClosedStaleNSQ(true);
        ASSERT_TRUE(ditags.getClosedStaleNSQ());
        ditags.setClosedStaleNSQ(false);
        ASSERT_FALSE(ditags.getClosedStaleNSQ());

        // Test Getters and Setters for Closed Stale APR
        // First test the default then test setting both ways
        ASSERT_FALSE(ditags.getClosedStaleAPR());
        ditags.setClosedStaleAPR(true);
        ASSERT_TRUE(ditags.getClosedStaleAPR());
        ditags.setClosedStaleAPR(false);
        ASSERT_FALSE(ditags.getClosedStaleAPR());

        // Test Getters and Setters for Pruned Stale NSQ
        // First test the default then test setting both ways
        ASSERT_FALSE(ditags.getPrunedStaleNSQ());
        ditags.setPrunedStaleNSQ(true);
        ASSERT_TRUE(ditags.getPrunedStaleNSQ());
        ditags.setPrunedStaleNSQ(false);
        ASSERT_FALSE(ditags.getPrunedStaleNSQ());

        // Test Getters and Setters for Pruned Stale APR
        // First test the default then test setting both ways
        ASSERT_FALSE(ditags.getPrunedStaleAPR());
        ditags.setPrunedStaleAPR(true);
        ASSERT_TRUE(ditags.getPrunedStaleAPR());
        ditags.setPrunedStaleAPR(false);
        ASSERT_FALSE(ditags.getPrunedStaleAPR());
    }

    TEST(DItags, WasNewAgentAdded)
    {
        std::ifstream fin_1("./data/task_allocation/itags_problem_inputs/full_run.json");
        nlohmann::json j_1;
        fin_1 >> j_1;
        std::shared_ptr<const ItagsProblemInputs> problem_inputs = j_1.get<std::shared_ptr<ItagsProblemInputs>>();

        std::ifstream fin_2("data/task_allocation/itags_problem_inputs/new_agent_test.json");
        nlohmann::json j_2;
        fin_2 >> j_2;
        std::shared_ptr<const ItagsProblemInputs> problem_inputs_new_agent =
            j_2.get<std::shared_ptr<ItagsProblemInputs>>();

        mocks::MockDitags ditags(problem_inputs_new_agent);

        bool was_agent_added = ditags.wasNewAgentAdded(problem_inputs);

        ASSERT_TRUE(was_agent_added);
    }

    TEST(DItags, AgentsLost)
    {
        std::ifstream fin_1("./data/task_allocation/itags_problem_inputs/full_run.json");
        nlohmann::json j_1;
        fin_1 >> j_1;
        std::shared_ptr<const ItagsProblemInputs> problem_inputs = j_1.get<std::shared_ptr<ItagsProblemInputs>>();

        std::ifstream fin_2("data/task_allocation/itags_problem_inputs/lost_agent_test.json");
        nlohmann::json j_2;
        fin_2 >> j_2;
        std::shared_ptr<const ItagsProblemInputs> problem_inputs_lost_agent =
            j_2.get<std::shared_ptr<ItagsProblemInputs>>();

        mocks::MockDitags ditags(problem_inputs_lost_agent);

        bool was_agent_lost = ditags.wasAgentLost(problem_inputs);
        ASSERT_TRUE(was_agent_lost);

        std::vector<int> agentsLost = ditags.agentsLost(problem_inputs);

        ASSERT_EQ(agentsLost.size(), 1);
    }

    TEST(DItags, DynNodeHash)
    {
        MatrixDimensions size = MatrixDimensions{.height = 3, .width = 3};
        auto node_root        = std::make_shared<DynIncrementalTaskAllocationNode>(size);

        Assignment alloc                             = Assignment{.task = 0, .robot = 1};
        DynIncrementalTaskAllocationNode node_second = DynIncrementalTaskAllocationNode(alloc, node_root);

        Assignment alloc2                                 = Assignment{.task = 0, .robot = 1};
        DynIncrementalTaskAllocationNode node_second_copy = DynIncrementalTaskAllocationNode(alloc2, node_root);

        auto h1 = node_second_copy.hash();
        auto h2 = node_second.hash();
        auto h3 = node_root->hash();

        ASSERT_EQ(node_second_copy.hash(), node_second.hash());

        ASSERT_NE(node_root->hash(), node_second.hash());

        ASSERT_EQ(node_root->hash(), node_root->hash());
    }

    TEST(DItags, AddPreviousSolutionToOpen)
    {
        std::ifstream fin("./data/task_allocation/itags_problem_inputs/full_run.json");
        nlohmann::json j;
        fin >> j;
        std::shared_ptr<const ItagsProblemInputs> problem_inputs = j.get<std::shared_ptr<ItagsProblemInputs>>();
        mocks::MockDitags ditags(problem_inputs);

        SearchResults<DynIncrementalTaskAllocationNode> results = ditags.search();
        std::cout << results.goal()->allocation() << std::endl;

        ditags.addPreviousSolutionToOpen();

        std::shared_ptr<DynIncrementalTaskAllocationNode> top_of_open = ditags.getOpen().pop();

        while(top_of_open->allocation() != results.goal()->allocation())
        {
            top_of_open = ditags.getOpen().pop();
        }
        ASSERT_EQ(results.goal()->allocation(), top_of_open->allocation());
    }

    TEST(DItags, RecomputeTetaqLocal)
    {
        std::ifstream fin("./data/task_allocation/itags_problem_inputs/full_run.json");
        nlohmann::json j;
        fin >> j;
        std::shared_ptr<const ItagsProblemInputs> problem_inputs = j.get<std::shared_ptr<ItagsProblemInputs>>();
        mocks::MockDitags ditags(problem_inputs);

        MatrixDimensions size = MatrixDimensions{.height = 3, .width = 3};
        auto node_root        = std::make_shared<DynIncrementalTaskAllocationNode>(size);

        node_root->setNSQ(0.5);
        node_root->setAPR(0.5);

        ditags.recomputeTETAQLocal(node_root);

        ASSERT_EQ(0.5, node_root->h());
    }

    TEST(DItags, AddNewNodesFromRoot)
    {
        std::ifstream fin("./data/task_allocation/itags_problem_inputs/full_run.json");
        nlohmann::json j;
        fin >> j;
        std::shared_ptr<const ItagsProblemInputs> problem_inputs = j.get<std::shared_ptr<ItagsProblemInputs>>();
        mocks::MockDitags ditags(problem_inputs);
        int oldSize = ditags.getOpen().size();

        std::ifstream fin_2("data/task_allocation/itags_problem_inputs/new_agent_test.json");
        nlohmann::json j_2;
        fin_2 >> j_2;
        std::shared_ptr<const ItagsProblemInputs> problem_inputs_new_agent =
            j_2.get<std::shared_ptr<ItagsProblemInputs>>();

        mocks::MockDitags ditags_new(problem_inputs);

        ditags_new.addNewNodesFromRoot(problem_inputs);

        int newSize = ditags_new.getOpen().size();

        ASSERT_EQ(oldSize, 0);
        ASSERT_EQ(newSize, 0);
    }

    TEST(DItags, UpdateFunctor)
    {
        std::ifstream fin_1("./data/task_allocation/itags_problem_inputs/full_run.json");
        nlohmann::json j_1;
        fin_1 >> j_1;
        std::shared_ptr<const ItagsProblemInputs> problem_inputs = j_1.get<std::shared_ptr<ItagsProblemInputs>>();

        std::ifstream fin_2("data/task_allocation/itags_problem_inputs/lost_agent_test.json");
        nlohmann::json j_2;
        fin_2 >> j_2;
        std::shared_ptr<const ItagsProblemInputs> problem_inputs_lost_agent =
            j_2.get<std::shared_ptr<ItagsProblemInputs>>();

        mocks::MockDitags ditags(problem_inputs);

        std::shared_ptr<const ItagsProblemInputs> oldFunctors = ditags.getItagsProblemInputs();

        ASSERT_EQ(oldFunctors->robots().size(), problem_inputs->robots().size());

        ditags.updateFunctors(problem_inputs_lost_agent, problem_inputs);

        std::shared_ptr<const ItagsProblemInputs> newFunctors = ditags.getItagsProblemInputs();

        ASSERT_EQ(newFunctors->robots()[1]->species()->name(), "lost_agent");
    }

    TEST(DItags, UpdateForLostAgent)
    {
        std::ifstream fin_1("data/task_allocation/itags_problem_inputs/new_agent_test.json");
        nlohmann::json j_1;
        fin_1 >> j_1;
        std::shared_ptr<const ItagsProblemInputs> problem_inputs = j_1.get<std::shared_ptr<ItagsProblemInputs>>();

        std::ifstream fin_2("data/task_allocation/itags_problem_inputs/all_agents_lost.json");
        nlohmann::json j_2;
        fin_2 >> j_2;
        std::shared_ptr<const ItagsProblemInputs> problem_inputs_lost_agent =
            j_2.get<std::shared_ptr<ItagsProblemInputs>>();

        mocks::MockDitags ditags(problem_inputs);

        SearchResults<DynIncrementalTaskAllocationNode> results = ditags.search();
        int oldSize                                             = ditags.getOpen().size();

        ditags.updateFunctors(problem_inputs_lost_agent, problem_inputs);

        ditags.updateForLostAgent(problem_inputs);

        int newSize = ditags.getOpen().size();

        ASSERT_NE(oldSize, newSize);
        ASSERT_EQ(newSize, 0);
    }

    TEST(DItags, TraitsUp)
    {
        std::ifstream fin_1("./data/task_allocation/itags_problem_inputs/full_run.json");
        nlohmann::json j_1;
        fin_1 >> j_1;
        std::shared_ptr<const ItagsProblemInputs> problem_inputs = j_1.get<std::shared_ptr<ItagsProblemInputs>>();

        std::ifstream fin_2("data/task_allocation/itags_problem_inputs/traits_up.json");
        nlohmann::json j_2;
        fin_2 >> j_2;
        std::shared_ptr<const ItagsProblemInputs> problem_inputs_req_trait_change =
            j_2.get<std::shared_ptr<ItagsProblemInputs>>();

        mocks::MockDitags ditags(problem_inputs_req_trait_change);

        bool needUpdateOpen   = ditags.needToUpdateOpenAPR(problem_inputs);
        bool needUpdateClosed = ditags.needToUpdateClosedAPR(problem_inputs);
        bool needUpdatePruned = ditags.needToUpdatePrunedAPR(problem_inputs);

        ASSERT_TRUE(needUpdateOpen);
        ASSERT_TRUE(needUpdateClosed);
        ASSERT_FALSE(needUpdatePruned);
    }

    TEST(DItags, TraitsDown)
    {
        std::ifstream fin_1("./data/task_allocation/itags_problem_inputs/full_run.json");
        nlohmann::json j_1;
        fin_1 >> j_1;
        std::shared_ptr<const ItagsProblemInputs> problem_inputs = j_1.get<std::shared_ptr<ItagsProblemInputs>>();

        std::ifstream fin_2("data/task_allocation/itags_problem_inputs/traits_down.json");
        nlohmann::json j_2;
        fin_2 >> j_2;
        std::shared_ptr<const ItagsProblemInputs> problem_inputs_req_trait_change =
            j_2.get<std::shared_ptr<ItagsProblemInputs>>();

        mocks::MockDitags ditags(problem_inputs_req_trait_change);

        bool needUpdateOpen   = ditags.needToUpdateOpenAPR(problem_inputs);
        bool needUpdateClosed = ditags.needToUpdateClosedAPR(problem_inputs);
        bool needUpdatePruned = ditags.needToUpdatePrunedAPR(problem_inputs);

        ASSERT_TRUE(needUpdateOpen);
        ASSERT_FALSE(needUpdateClosed);
        ASSERT_TRUE(needUpdatePruned);
    }

    TEST(DItags, TraitsUpAndDown)
    {
        std::ifstream fin_1("./data/task_allocation/itags_problem_inputs/full_run.json");
        nlohmann::json j_1;
        fin_1 >> j_1;
        std::shared_ptr<const ItagsProblemInputs> problem_inputs = j_1.get<std::shared_ptr<ItagsProblemInputs>>();

        std::ifstream fin_2("data/task_allocation/itags_problem_inputs/traits_up_and_down.json");
        nlohmann::json j_2;
        fin_2 >> j_2;
        std::shared_ptr<const ItagsProblemInputs> problem_inputs_req_trait_change =
            j_2.get<std::shared_ptr<ItagsProblemInputs>>();

        mocks::MockDitags ditags(problem_inputs_req_trait_change);

        bool needUpdateOpen   = ditags.needToUpdateOpenAPR(problem_inputs);
        bool needUpdateClosed = ditags.needToUpdateClosedAPR(problem_inputs);
        bool needUpdatePruned = ditags.needToUpdatePrunedAPR(problem_inputs);

        ASSERT_TRUE(needUpdateOpen);
        ASSERT_TRUE(needUpdateClosed);
        ASSERT_TRUE(needUpdatePruned);
    }

    TEST(DItags, ReqsUp)
    {
        std::ifstream fin_1("./data/task_allocation/itags_problem_inputs/full_run.json");
        nlohmann::json j_1;
        fin_1 >> j_1;
        std::shared_ptr<const ItagsProblemInputs> problem_inputs = j_1.get<std::shared_ptr<ItagsProblemInputs>>();

        std::ifstream fin_2("data/task_allocation/itags_problem_inputs/reqs_up.json");
        nlohmann::json j_2;
        fin_2 >> j_2;
        std::shared_ptr<const ItagsProblemInputs> problem_inputs_req_trait_change =
            j_2.get<std::shared_ptr<ItagsProblemInputs>>();

        mocks::MockDitags ditags(problem_inputs_req_trait_change);

        bool needUpdateOpen   = ditags.needToUpdateOpenAPR(problem_inputs);
        bool needUpdateClosed = ditags.needToUpdateClosedAPR(problem_inputs);
        bool needUpdatePruned = ditags.needToUpdatePrunedAPR(problem_inputs);

        ASSERT_TRUE(needUpdateOpen);
        ASSERT_FALSE(needUpdateClosed);
        ASSERT_TRUE(needUpdatePruned);
    }

    TEST(DItags, ReqsDown)
    {
        std::ifstream fin_1("./data/task_allocation/itags_problem_inputs/full_run.json");
        nlohmann::json j_1;
        fin_1 >> j_1;
        std::shared_ptr<const ItagsProblemInputs> problem_inputs = j_1.get<std::shared_ptr<ItagsProblemInputs>>();

        std::ifstream fin_2("data/task_allocation/itags_problem_inputs/reqs_down.json");
        nlohmann::json j_2;
        fin_2 >> j_2;
        std::shared_ptr<const ItagsProblemInputs> problem_inputs_req_trait_change =
            j_2.get<std::shared_ptr<ItagsProblemInputs>>();

        mocks::MockDitags ditags(problem_inputs_req_trait_change);

        bool needUpdateOpen   = ditags.needToUpdateOpenAPR(problem_inputs);
        bool needUpdateClosed = ditags.needToUpdateClosedAPR(problem_inputs);
        bool needUpdatePruned = ditags.needToUpdatePrunedAPR(problem_inputs);

        ASSERT_TRUE(needUpdateOpen);
        ASSERT_TRUE(needUpdateClosed);
        ASSERT_FALSE(needUpdatePruned);
    }

    TEST(DItags, ReqsUpAndDown)
    {
        std::ifstream fin_1("./data/task_allocation/itags_problem_inputs/full_run.json");
        nlohmann::json j_1;
        fin_1 >> j_1;
        std::shared_ptr<const ItagsProblemInputs> problem_inputs = j_1.get<std::shared_ptr<ItagsProblemInputs>>();

        std::ifstream fin_2("data/task_allocation/itags_problem_inputs/reqs_up_and_down.json");
        nlohmann::json j_2;
        fin_2 >> j_2;
        std::shared_ptr<const ItagsProblemInputs> problem_inputs_req_trait_change =
            j_2.get<std::shared_ptr<ItagsProblemInputs>>();

        mocks::MockDitags ditags(problem_inputs_req_trait_change);

        bool needUpdateOpen   = ditags.needToUpdateOpenAPR(problem_inputs);
        bool needUpdateClosed = ditags.needToUpdateClosedAPR(problem_inputs);
        bool needUpdatePruned = ditags.needToUpdatePrunedAPR(problem_inputs);

        ASSERT_TRUE(needUpdateOpen);
        ASSERT_TRUE(needUpdateClosed);
        ASSERT_TRUE(needUpdatePruned);
    }

    TEST(DItags, ReqsAndTraitsUnchanged)
    {
        std::ifstream fin_1("./data/task_allocation/itags_problem_inputs/full_run.json");
        nlohmann::json j_1;
        fin_1 >> j_1;
        std::shared_ptr<const ItagsProblemInputs> problem_inputs = j_1.get<std::shared_ptr<ItagsProblemInputs>>();

        std::ifstream fin_2("./data/task_allocation/itags_problem_inputs/full_run.json");
        nlohmann::json j_2;
        fin_2 >> j_2;
        std::shared_ptr<const ItagsProblemInputs> problem_inputs_req_trait_change =
            j_2.get<std::shared_ptr<ItagsProblemInputs>>();

        mocks::MockDitags ditags(problem_inputs_req_trait_change);

        bool needUpdateOpen   = ditags.needToUpdateOpenAPR(problem_inputs);
        bool needUpdateClosed = ditags.needToUpdateClosedAPR(problem_inputs);
        bool needUpdatePruned = ditags.needToUpdatePrunedAPR(problem_inputs);

        ASSERT_FALSE(needUpdateOpen);
        ASSERT_FALSE(needUpdateClosed);
        ASSERT_FALSE(needUpdatePruned);
    }

    TEST(DItags, Unchanged_sched)
    {
        std::ifstream fin_1("./data/task_allocation/itags_problem_inputs/full_run.json");
        nlohmann::json j_1;
        fin_1 >> j_1;
        std::shared_ptr<const ItagsProblemInputs> problem_inputs = j_1.get<std::shared_ptr<ItagsProblemInputs>>();

        std::ifstream fin_2("./data/task_allocation/itags_problem_inputs/full_run.json");
        nlohmann::json j_2;
        fin_2 >> j_2;
        std::shared_ptr<const ItagsProblemInputs> problem_inputs_req_trait_change =
            j_2.get<std::shared_ptr<ItagsProblemInputs>>();

        mocks::MockDitags ditags(problem_inputs_req_trait_change);

        bool needUpdateOpen = ditags.needToUpdateOpenNSQ(problem_inputs);

        ASSERT_FALSE(needUpdateOpen);
    }

    TEST(DItags, DurationChange)
    {
        std::ifstream fin_1("./data/task_allocation/itags_problem_inputs/full_run.json");
        nlohmann::json j_1;
        fin_1 >> j_1;
        std::shared_ptr<const ItagsProblemInputs> problem_inputs = j_1.get<std::shared_ptr<ItagsProblemInputs>>();

        std::ifstream fin_2("data/task_allocation/itags_problem_inputs/duration_change.json");
        nlohmann::json j_2;
        fin_2 >> j_2;
        std::shared_ptr<const ItagsProblemInputs> problem_inputs_req_trait_change =
            j_2.get<std::shared_ptr<ItagsProblemInputs>>();

        mocks::MockDitags ditags(problem_inputs_req_trait_change);

        bool needUpdateOpen = ditags.needToUpdateOpenNSQ(problem_inputs);

        ASSERT_TRUE(needUpdateOpen);
    }

    TEST(DItags, PrecedenceConstraintsChange)
    {
        std::ifstream fin_1("./data/task_allocation/itags_problem_inputs/full_run.json");
        nlohmann::json j_1;
        fin_1 >> j_1;
        std::shared_ptr<const ItagsProblemInputs> problem_inputs = j_1.get<std::shared_ptr<ItagsProblemInputs>>();

        std::ifstream fin_2("data/task_allocation/itags_problem_inputs/precedence_change.json");
        nlohmann::json j_2;
        fin_2 >> j_2;
        std::shared_ptr<const ItagsProblemInputs> problem_inputs_req_trait_change =
            j_2.get<std::shared_ptr<ItagsProblemInputs>>();

        mocks::MockDitags ditags(problem_inputs_req_trait_change);

        bool needUpdateOpen = ditags.needToUpdateOpenNSQ(problem_inputs);

        ASSERT_TRUE(needUpdateOpen);
    }

    TEST(DItags, UpdateOpenAPR)
    {
        std::ifstream fin_2("data/task_allocation/itags_problem_inputs/no_reqs.json");
        nlohmann::json j_2;
        fin_2 >> j_2;
        std::shared_ptr<const ItagsProblemInputs> problem_inputs = j_2.get<std::shared_ptr<ItagsProblemInputs>>();

        mocks::MockDitags ditags(problem_inputs);

        MatrixDimensions size = MatrixDimensions{.height = 3, .width = 2};
        auto node_root        = std::make_shared<DynIncrementalTaskAllocationNode>(size);

        Assignment alloc = Assignment{.task = 0, .robot = 1};
        auto node_second = std::make_shared<DynIncrementalTaskAllocationNode>(alloc, node_root);

        Assignment alloc2 = Assignment{.task = 1, .robot = 1};
        auto node_third   = std::make_shared<DynIncrementalTaskAllocationNode>(alloc2, node_root);

        Assignment alloc3 = Assignment{.task = 2, .robot = 1};
        auto node_fourth  = std::make_shared<DynIncrementalTaskAllocationNode>(alloc3, node_root);
        // set APR

        node_root->setAPR(1);
        node_root->setNSQ(1);

        node_second->setAPR(1);
        node_second->setNSQ(1);

        node_third->setAPR(1);
        node_third->setNSQ(1);

        node_fourth->setAPR(1);
        node_fourth->setNSQ(1);

        ditags.addNodeToOpen(node_root);
        ditags.addNodeToOpen(node_second);
        ditags.addNodeToOpen(node_third);
        ditags.addNodeToOpen(node_fourth);

        ditags.updateNodesOpenAPR();

        std::shared_ptr<DynIncrementalTaskAllocationNode> node_top_open    = ditags.popOpen();
        std::shared_ptr<DynIncrementalTaskAllocationNode> node_second_open = ditags.popOpen();
        std::shared_ptr<DynIncrementalTaskAllocationNode> node_third_open  = ditags.popOpen();
        std::shared_ptr<DynIncrementalTaskAllocationNode> node_fourth_open = ditags.popOpen();

        ASSERT_LT(node_top_open->getAPR().value() - float(1 - 0.2 / (0.2 + 0.15 + 0.1)), 0.01);
        ASSERT_LT(node_second_open->getAPR().value() - float(1 - 0.15 / (0.2 + 0.15 + 0.1)), 0.01);
        ASSERT_LT(node_third_open->getAPR().value() - float(1 - 0.1 / (0.2 + 0.15 + 0.1)), 0.01);
        ASSERT_LT(node_fourth_open->getAPR().value() - float(1 - 0.0 / (0.2 + 0.15 + 0.1)), 0.01);
    }

    TEST(DItags, UpdateOpenNSQ)
    {
        std::ifstream fin_2("data/task_allocation/itags_problem_inputs/no_reqs.json");
        nlohmann::json j_2;
        fin_2 >> j_2;
        std::shared_ptr<const ItagsProblemInputs> problem_inputs = j_2.get<std::shared_ptr<ItagsProblemInputs>>();

        mocks::MockDitags ditags(problem_inputs);

        MatrixDimensions size = MatrixDimensions{.height = 3, .width = 2};
        auto node_root        = std::make_shared<DynIncrementalTaskAllocationNode>(size);

        Assignment alloc = Assignment{.task = 0, .robot = 1};
        auto node_second = std::make_shared<DynIncrementalTaskAllocationNode>(alloc, node_root);

        Assignment alloc2 = Assignment{.task = 1, .robot = 1};
        auto node_third   = std::make_shared<DynIncrementalTaskAllocationNode>(alloc2, node_root);

        Assignment alloc3 = Assignment{.task = 2, .robot = 1};
        auto node_fourth  = std::make_shared<DynIncrementalTaskAllocationNode>(alloc3, node_root);
        // set APR

        node_root->setAPR(1);
        node_root->setNSQ(1);

        node_second->setAPR(1);
        node_second->setNSQ(1);

        node_third->setAPR(1);
        node_third->setNSQ(1);

        node_fourth->setAPR(1);
        node_fourth->setNSQ(1);

        ditags.addNodeToOpen(node_root);
        ditags.addNodeToOpen(node_second);
        ditags.addNodeToOpen(node_third);
        ditags.addNodeToOpen(node_fourth);

        ditags.updateNodesOpenNSQ();

        std::shared_ptr<DynIncrementalTaskAllocationNode> node_top_open    = ditags.popOpen();
        std::shared_ptr<DynIncrementalTaskAllocationNode> node_second_open = ditags.popOpen();
        std::shared_ptr<DynIncrementalTaskAllocationNode> node_third_open  = ditags.popOpen();
        std::shared_ptr<DynIncrementalTaskAllocationNode> node_fourth_open = ditags.popOpen();

        ASSERT_EQ(node_top_open->getNSQ().value(), 0.0);
        ASSERT_LT(node_second_open->getNSQ().value(), 0.009);
        ASSERT_LT(node_third_open->getNSQ().value(), 0.009);
        ASSERT_LT(node_fourth_open->getNSQ().value(), 0.009);
    }

    TEST(DItags, UpdateClosedAPRDeep)
    {
        std::ifstream fin_2("data/task_allocation/itags_problem_inputs/no_reqs.json");
        nlohmann::json j_2;
        fin_2 >> j_2;
        std::shared_ptr<const ItagsProblemInputs> problem_inputs = j_2.get<std::shared_ptr<ItagsProblemInputs>>();

        mocks::MockDitags ditags(problem_inputs);

        MatrixDimensions size = MatrixDimensions{.height = 3, .width = 2};
        auto node_root        = std::make_shared<DynIncrementalTaskAllocationNode>(size);

        Assignment alloc = Assignment{.task = 0, .robot = 1};
        auto node_second = std::make_shared<DynIncrementalTaskAllocationNode>(alloc, node_root);

        Assignment alloc2 = Assignment{.task = 1, .robot = 1};
        auto node_third   = std::make_shared<DynIncrementalTaskAllocationNode>(alloc2, node_root);

        Assignment alloc3 = Assignment{.task = 2, .robot = 1};
        auto node_fourth  = std::make_shared<DynIncrementalTaskAllocationNode>(alloc3, node_root);
        // set APR

        node_root->setAPR(1);
        node_root->setNSQ(1);

        node_second->setAPR(1);
        node_second->setNSQ(1);

        node_third->setAPR(1);
        node_third->setNSQ(1);

        node_fourth->setAPR(1);
        node_fourth->setNSQ(1);

        ditags.addNodeToClosed(node_root);
        ditags.addNodeToClosed(node_second);
        ditags.addNodeToClosed(node_third);
        ditags.addNodeToClosed(node_fourth);

        ditags.updateNodesClosedAPR();

        std::vector<std::shared_ptr<DynIncrementalTaskAllocationNode>> closed = ditags.getClosed();

        ASSERT_TRUE(closed.empty());
    }

    TEST(DItags, UpdatePrunedAPRDeep)
    {
        std::ifstream fin_2("data/task_allocation/itags_problem_inputs/no_reqs.json");
        nlohmann::json j_2;
        fin_2 >> j_2;
        std::shared_ptr<const ItagsProblemInputs> problem_inputs = j_2.get<std::shared_ptr<ItagsProblemInputs>>();

        mocks::MockDitags ditags(problem_inputs);

        MatrixDimensions size = MatrixDimensions{.height = 3, .width = 2};
        auto node_root        = std::make_shared<DynIncrementalTaskAllocationNode>(size);

        Assignment alloc = Assignment{
            .task  = 0,
            .robot = 1,
        };
        auto node_second = std::make_shared<DynIncrementalTaskAllocationNode>(alloc, node_root);

        Assignment alloc2 = Assignment{.task = 1, .robot = 1};
        auto node_third   = std::make_shared<DynIncrementalTaskAllocationNode>(alloc2, node_root);

        Assignment alloc3 = Assignment{
            .task  = 2,
            .robot = 1,
        };
        auto node_fourth = std::make_shared<DynIncrementalTaskAllocationNode>(alloc3, node_root);
        // set APR

        node_root->setAPR(1);
        node_root->setNSQ(1);

        node_second->setAPR(1);
        node_second->setNSQ(1);

        node_third->setAPR(1);
        node_third->setNSQ(1);

        node_fourth->setAPR(1);
        node_fourth->setNSQ(1);

        ditags.addNodeToPruned(node_root);
        ditags.addNodeToPruned(node_second);
        ditags.addNodeToPruned(node_third);
        ditags.addNodeToPruned(node_fourth);

        ditags.updateNodesPrunedAPR();

        std::vector<std::shared_ptr<DynIncrementalTaskAllocationNode>> pruned = ditags.getPruned();

        ASSERT_TRUE(pruned.empty());
    }

    TEST(DItags, DeepCopyEasy)
    {
        std::ifstream fin_2("data/task_allocation/itags_problem_inputs/no_reqs.json");
        nlohmann::json j_2;
        fin_2 >> j_2;
        std::shared_ptr<const ItagsProblemInputs> problem_inputs = j_2.get<std::shared_ptr<ItagsProblemInputs>>();

        mocks::MockDitags ditags(problem_inputs);

        MatrixDimensions size = MatrixDimensions{.height = 3, .width = 2};
        auto node_root        = std::make_shared<DynIncrementalTaskAllocationNode>(size);

        Assignment alloc = Assignment{
            .task  = 0,
            .robot = 1,
        };
        auto node_second = std::make_shared<DynIncrementalTaskAllocationNode>(alloc, node_root);

        Assignment alloc2 = Assignment{
            .task  = 1,
            .robot = 1,
        };
        auto node_third = std::make_shared<DynIncrementalTaskAllocationNode>(alloc2, node_root);

        Assignment alloc3 = Assignment{
            .task  = 2,
            .robot = 1,
        };
        auto node_fourth = std::make_shared<DynIncrementalTaskAllocationNode>(alloc3, node_root);
        // set APR

        node_root->setAPR(1);
        node_root->setNSQ(1);

        node_second->setAPR(1);
        node_second->setNSQ(1);

        node_third->setAPR(1);
        node_third->setNSQ(1);

        node_fourth->setAPR(1);
        node_fourth->setNSQ(1);

        ditags.addNodeToClosed(node_root);
        ditags.addNodeToOpen(node_second);
        ditags.addNodeToOpen(node_third);
        ditags.addNodeToPruned(node_fourth);

        mocks::MockDitags ditags_copy(ditags);

        std::shared_ptr<DynIncrementalTaskAllocationNode> node_top_open    = ditags_copy.popOpen();
        std::shared_ptr<DynIncrementalTaskAllocationNode> node_second_open = ditags_copy.popOpen();

        auto closed = ditags_copy.getClosed();
        auto pruned = ditags_copy.getPruned();

        ASSERT_EQ(node_top_open->allocation(), node_third->allocation());
        ASSERT_EQ(node_second_open->allocation(), node_second->allocation());
        ASSERT_EQ(closed.back()->allocation(), node_root->allocation());
        ASSERT_EQ(pruned.back()->allocation(), node_fourth->allocation());
    }

    TEST(DItags, DeepCopyHard)
    {
        std::ifstream fin_2("data/task_allocation/itags_problem_inputs/no_reqs.json");
        nlohmann::json j_2;
        fin_2 >> j_2;
        std::shared_ptr<const ItagsProblemInputs> problem_inputs = j_2.get<std::shared_ptr<ItagsProblemInputs>>();

        mocks::MockDitags ditags(problem_inputs);

        MatrixDimensions size = MatrixDimensions{
            .height = 3,
            .width  = 2,
        };
        auto node_root = std::make_shared<DynIncrementalTaskAllocationNode>(size);
        node_root->setAPR(1);
        node_root->setNSQ(1);
        ditags.addNodeToClosed(node_root);

        Assignment alloc = Assignment{.task = 0, .robot = 1};
        auto node_second = std::make_shared<DynIncrementalTaskAllocationNode>(alloc, node_root);
        node_second->setAPR(1);
        node_second->setNSQ(1);
        ditags.addNodeToClosed(node_second);

        Assignment alloc2 = Assignment{.task = 0, .robot = 0};
        auto node_third   = std::make_shared<DynIncrementalTaskAllocationNode>(alloc2, node_root);
        node_third->setAPR(1);
        node_third->setNSQ(1);
        ditags.addNodeToClosed(node_third);

        Assignment alloc3 = Assignment{.task = 0, .robot = 0};
        auto node_fourth  = std::make_shared<DynIncrementalTaskAllocationNode>(alloc3, node_second);
        node_fourth->setAPR(1);
        node_fourth->setNSQ(1);
        ditags.addNodeToOpen(node_fourth);
        ASSERT_EQ(ditags.getOpen().size(), 1);

        Assignment alloc4 = Assignment{.task = 1, .robot = 0};
        auto node_fifth   = std::make_shared<DynIncrementalTaskAllocationNode>(alloc4, node_second);
        node_fifth->setAPR(1);
        node_fifth->setNSQ(1);
        ditags.addNodeToOpen(node_fifth);
        ASSERT_EQ(ditags.getOpen().size(), 2);

        Assignment alloc5 = Assignment{.task = 0, .robot = 1};
        auto node_sixth   = std::make_shared<DynIncrementalTaskAllocationNode>(alloc5, node_third);
        node_sixth->setAPR(1);
        node_sixth->setNSQ(1);
        ditags.addNodeToClosed(node_sixth);

        Assignment alloc6 = Assignment{.task = 1, .robot = 1};
        auto node_seventh = std::make_shared<DynIncrementalTaskAllocationNode>(alloc6, node_third);
        node_seventh->setAPR(1);
        node_seventh->setNSQ(1);
        ditags.addNodeToPruned(node_seventh);

        Assignment alloc7 = Assignment{
            .task  = 2,
            .robot = 1,
        };
        auto node_eight = std::make_shared<DynIncrementalTaskAllocationNode>(alloc7, node_sixth);
        node_eight->setAPR(1);
        node_eight->setNSQ(1);
        ditags.addNodeToOpen(node_eight);
        ASSERT_EQ(ditags.getOpen().size(), 3);
        // set APR

        mocks::MockDitags ditags_copy(ditags);

        auto open   = ditags_copy.getOpen();
        auto closed = ditags_copy.getClosed();
        auto pruned = ditags_copy.getPruned();

        ASSERT_EQ(closed.size(), 4);
        ASSERT_EQ(pruned.size(), 1);
        ASSERT_EQ(ditags_copy.getOpen().size(), 3);

        ASSERT_EQ(pruned[0]->allocation(), node_seventh->allocation());

        ASSERT_EQ(closed[0]->allocation(), node_root->allocation());
        ASSERT_EQ(closed[1]->allocation(), node_third->allocation());
        ASSERT_EQ(closed[2]->allocation(), node_sixth->allocation());
        ASSERT_EQ(closed[3]->allocation(), node_second->allocation());

        ASSERT_EQ(ditags_copy.popOpen()->allocation(), node_eight->allocation());
        ASSERT_EQ(ditags_copy.popOpen()->allocation(), node_fifth->allocation());
        ASSERT_EQ(ditags_copy.popOpen()->allocation(), node_fourth->allocation());
    }

    TEST(DItags, ShallowCopyEasy)
    {
        std::ifstream fin_2("data/task_allocation/itags_problem_inputs/no_reqs.json");
        nlohmann::json j_2;
        fin_2 >> j_2;
        std::shared_ptr<const ItagsProblemInputs> problem_inputs = j_2.get<std::shared_ptr<ItagsProblemInputs>>();

        std::shared_ptr<mocks::MockDitags> ditags = std::make_shared<mocks::MockDitags>(problem_inputs);

        MatrixDimensions size = MatrixDimensions{.height = 3, .width = 2};
        auto node_root        = std::make_shared<DynIncrementalTaskAllocationNode>(size);

        Assignment alloc = Assignment{
            .task  = 0,
            .robot = 1,
        };
        auto node_second = std::make_shared<DynIncrementalTaskAllocationNode>(alloc, node_root);

        Assignment alloc2 = Assignment{.task = 1, .robot = 1};
        auto node_third   = std::make_shared<DynIncrementalTaskAllocationNode>(alloc2, node_root);

        Assignment alloc3 = Assignment{
            .task  = 2,
            .robot = 1,
        };
        auto node_fourth = std::make_shared<DynIncrementalTaskAllocationNode>(alloc3, node_root);
        // set APR

        node_root->setAPR(1);
        node_root->setNSQ(1);

        node_second->setAPR(1);
        node_second->setNSQ(1);

        node_third->setAPR(1);
        node_third->setNSQ(1);

        node_fourth->setAPR(1);
        node_fourth->setNSQ(1);

        ditags->addNodeToClosed(node_root);
        ditags->addNodeToOpen(node_second);
        ditags->addNodeToOpen(node_third);
        ditags->addNodeToPruned(node_fourth);

        mocks::MockDitags ditags_copy(*ditags, false);

        std::shared_ptr<DynIncrementalTaskAllocationNode> node_top_open    = ditags_copy.popOpen();
        std::shared_ptr<DynIncrementalTaskAllocationNode> node_second_open = ditags_copy.popOpen();

        auto closed = ditags_copy.getClosedID();
        auto pruned = ditags_copy.getPrunedID();

        ASSERT_EQ(node_top_open->allocation(), node_third->allocation());
        ASSERT_EQ(node_second_open->allocation(), node_second->allocation());
        ASSERT_EQ(closed.size(), 1);
        ASSERT_EQ(pruned.size(), 1);
    }

    TEST(DItags, ShallowCopyHard)
    {
        std::ifstream fin_2("data/task_allocation/itags_problem_inputs/no_reqs.json");
        nlohmann::json j_2;
        fin_2 >> j_2;
        std::shared_ptr<const ItagsProblemInputs> problem_inputs = j_2.get<std::shared_ptr<ItagsProblemInputs>>();

        mocks::MockDitags ditags(problem_inputs);

        MatrixDimensions size = MatrixDimensions{.height = 3, .width = 2};
        auto node_root        = std::make_shared<DynIncrementalTaskAllocationNode>(size);
        node_root->setAPR(1);
        node_root->setNSQ(1);
        ditags.addNodeToClosed(node_root);

        Assignment alloc = Assignment{.task = 0, .robot = 1};
        auto node_second = std::make_shared<DynIncrementalTaskAllocationNode>(alloc, node_root);
        node_second->setAPR(1);
        node_second->setNSQ(1);
        ditags.addNodeToClosed(node_second);

        Assignment alloc2 = Assignment{.task = 0, .robot = 0};
        auto node_third   = std::make_shared<DynIncrementalTaskAllocationNode>(alloc2, node_root);
        node_third->setAPR(1);
        node_third->setNSQ(1);
        ditags.addNodeToClosed(node_third);

        Assignment alloc3 = Assignment{
            .task  = 0,
            .robot = 0,
        };
        auto node_fourth = std::make_shared<DynIncrementalTaskAllocationNode>(alloc3, node_second);
        node_fourth->setAPR(1);
        node_fourth->setNSQ(1);
        ditags.addNodeToOpen(node_fourth);
        ASSERT_EQ(ditags.getOpen().size(), 1);

        Assignment alloc4 = Assignment{.task = 1, .robot = 0};
        auto node_fifth   = std::make_shared<DynIncrementalTaskAllocationNode>(alloc4, node_second);
        node_fifth->setAPR(1);
        node_fifth->setNSQ(1);
        ditags.addNodeToOpen(node_fifth);
        ASSERT_EQ(ditags.getOpen().size(), 2);

        Assignment alloc5 = Assignment{.task = 1, .robot = 0};
        auto node_sixth   = std::make_shared<DynIncrementalTaskAllocationNode>(alloc5, node_third);
        node_sixth->setAPR(1);
        node_sixth->setNSQ(1);
        ditags.addNodeToClosed(node_sixth);

        Assignment alloc6 = Assignment{.task = 1, .robot = 1};
        auto node_seventh = std::make_shared<DynIncrementalTaskAllocationNode>(alloc6, node_third);
        node_seventh->setAPR(1);
        node_seventh->setNSQ(1);
        ditags.addNodeToPruned(node_seventh);

        Assignment alloc7 = Assignment{.task = 2, .robot = 1};
        auto node_eight   = std::make_shared<DynIncrementalTaskAllocationNode>(alloc7, node_sixth);
        node_eight->setAPR(1);
        node_eight->setNSQ(1);
        ditags.addNodeToOpen(node_eight);
        ASSERT_EQ(ditags.getOpen().size(), 3);
        // set APR

        mocks::MockDitags ditags_copy(ditags, false);

        auto open   = ditags_copy.getOpen();
        auto closed = ditags_copy.getClosedID();
        auto pruned = ditags_copy.getPrunedID();

        ASSERT_EQ(closed.size(), 4);
        ASSERT_EQ(pruned.size(), 1);
        ASSERT_EQ(ditags_copy.getOpen().size(), 3);

        ASSERT_EQ(ditags_copy.popOpen()->allocation(), node_eight->allocation());
        ASSERT_EQ(ditags_copy.popOpen()->allocation(), node_fifth->allocation());
        ASSERT_EQ(ditags_copy.popOpen()->allocation(), node_fourth->allocation());
    }

    TEST(DItags, UpdateClosedAPRShallow)
    {
        std::ifstream fin_2("data/task_allocation/itags_problem_inputs/all_sol.json");
        nlohmann::json j_2;
        fin_2 >> j_2;
        std::shared_ptr<const ItagsProblemInputs> problem_inputs = j_2.get<std::shared_ptr<ItagsProblemInputs>>();

        mocks::MockDitags ditags(problem_inputs);

        MatrixDimensions size = MatrixDimensions{.height = 3, .width = 2};
        auto node_root        = std::make_shared<DynIncrementalTaskAllocationNode>(size);

        Assignment alloc = Assignment{.task = 0, .robot = 1};
        auto node_second = std::make_shared<DynIncrementalTaskAllocationNode>(alloc, node_root);

        Assignment alloc2 = Assignment{.task = 1, .robot = 1};
        auto node_third   = std::make_shared<DynIncrementalTaskAllocationNode>(alloc2, node_root);

        Assignment alloc3 = Assignment{.task = 2, .robot = 1};
        auto node_fourth  = std::make_shared<DynIncrementalTaskAllocationNode>(alloc3, node_root);
        // set APR

        node_root->setAPR(1);
        node_root->setNSQ(1);

        node_second->setAPR(1);
        node_second->setNSQ(1);

        node_third->setAPR(1);
        node_third->setNSQ(1);

        node_fourth->setAPR(1);
        node_fourth->setNSQ(1);

        ditags.addNodeToClosed(node_root);
        ditags.addNodeToClosed(node_second);
        ditags.addNodeToClosed(node_third);
        ditags.addNodeToClosed(node_fourth);

        mocks::MockDitags ditags_copy(ditags, false);

        ditags_copy.updateNodesClosedAPR();

        std::vector<std::shared_ptr<DynIncrementalTaskAllocationNode>> closed = ditags_copy.getClosed();

        auto open_top    = ditags_copy.popOpen();
        auto open_second = ditags_copy.popOpen();
        auto open_third  = ditags_copy.popOpen();
        auto open_fourth = ditags_copy.popOpen();

        ASSERT_EQ(open_fourth->allocation(), node_root->allocation());
        ASSERT_EQ(open_third->allocation(), node_second->allocation());
        ASSERT_EQ(open_second->allocation(), node_third->allocation());
        ASSERT_EQ(open_top->allocation(), node_fourth->allocation());
    }

    TEST(DItags, UpdatePrunedAPRShallow)
    {
        std::ifstream fin_2("data/task_allocation/itags_problem_inputs/all_sol.json");
        nlohmann::json j_2;
        fin_2 >> j_2;
        std::shared_ptr<const ItagsProblemInputs> problem_inputs = j_2.get<std::shared_ptr<ItagsProblemInputs>>();

        mocks::MockDitags ditags(problem_inputs);

        MatrixDimensions size = MatrixDimensions{.height = 3, .width = 2};
        auto node_root        = std::make_shared<DynIncrementalTaskAllocationNode>(size);

        Assignment alloc = Assignment{.task = 0, .robot = 1};
        auto node_second = std::make_shared<DynIncrementalTaskAllocationNode>(alloc, node_root);

        Assignment alloc2 = Assignment{.task = 1, .robot = 1};
        auto node_third   = std::make_shared<DynIncrementalTaskAllocationNode>(alloc2, node_root);

        Assignment alloc3 = Assignment{.task = 2, .robot = 1};
        auto node_fourth  = std::make_shared<DynIncrementalTaskAllocationNode>(alloc3, node_root);
        // set APR

        node_root->setAPR(1);
        node_root->setNSQ(1);

        node_second->setAPR(1);
        node_second->setNSQ(1);

        node_third->setAPR(1);
        node_third->setNSQ(1);

        node_fourth->setAPR(1);
        node_fourth->setNSQ(1);

        ditags.addNodeToPruned(node_root);
        ditags.addNodeToPruned(node_second);
        ditags.addNodeToPruned(node_third);
        ditags.addNodeToPruned(node_fourth);

        mocks::MockDitags ditags_copy(ditags, false);

        ditags_copy.updateNodesPrunedAPR();

        std::vector<std::shared_ptr<DynIncrementalTaskAllocationNode>> closed = ditags_copy.getPruned();

        auto open_top = ditags_copy.popOpen();

        ASSERT_EQ(open_top->allocation(), node_fourth->allocation());
    }

    TEST(DItags, Repair)
    {
        std::ifstream fin("./data/task_allocation/itags_problem_inputs/full_run.json");
        nlohmann::json j;
        fin >> j;
        std::shared_ptr<const ItagsProblemInputs> problem_inputs = j.get<std::shared_ptr<ItagsProblemInputs>>();
        mocks::MockDitags ditags(problem_inputs);

        std::ifstream fin_2("./data/task_allocation/itags_problem_inputs/duration_change.json");
        nlohmann::json j_2;
        fin_2 >> j_2;
        std::shared_ptr<const ItagsProblemInputs> problem_inputs_req_trait_change =
            j_2.get<std::shared_ptr<ItagsProblemInputs>>();

        SearchResults<DynIncrementalTaskAllocationNode> results = ditags.search();

        mocks::MockDitags ditag_copy(ditags, true);
        ditag_copy.repairSearch(problem_inputs_req_trait_change);
        SearchResults<DynIncrementalTaskAllocationNode> results_repair = ditag_copy.continueSearch();
        ASSERT_EQ(results_repair.goal()->allocation(), results.goal()->allocation());
    }

    //    TEST(DItags, RepairShallow)
    //    {
    //        std::ifstream fin("./data/task_allocation/itags_problem_inputs/full_run.json");
    //        nlohmann::json j;
    //        fin >> j;
    //        std::shared_ptr<const ItagsProblemInputs> problem_inputs = j.get<std::shared_ptr<ItagsProblemInputs>>();
    //        mocks::MockDitags ditags(problem_inputs);
    //
    //        std::ifstream fin_2("./data/task_allocation/itags_problem_inputs/traits_up.json");
    //        nlohmann::json j_2;
    //        fin_2 >> j_2;
    //        std::shared_ptr<const ItagsProblemInputs> problem_inputs_req_trait_change =
    //            j_2.get<std::shared_ptr<ItagsProblemInputs>>();
    //
    //        SearchResults<DynIncrementalTaskAllocationNode> results = ditags.search();
    //        ditags.writeSolutionToFile("test_itags_output.json", results.goal());
    //        mocks::MockDitags ditags_copy(ditags, false);
    //
    //        ditags_copy.repairSearch(problem_inputs_req_trait_change);
    //        SearchResults<DynIncrementalTaskAllocationNode> results_repair = ditags_copy.continueSearch();
    //        ditags_copy.writeSolutionToFile("test_itags_repair_output.json", results_repair.goal());
    //        ASSERT_EQ(results_repair.goal()->allocation(), results.goal()->allocation());
    //    }

    TEST(DItags, TestSurvivor)
    {
        std::filesystem::path cwd = std::filesystem::current_path();
        std::cout << cwd;

        std::ifstream fin("./data/task_allocation/itags_problem_inputs/test_survivor.json");
        nlohmann::json j;
        fin >> j;
        std::shared_ptr<const ItagsProblemInputs> problem_inputs = j.get<std::shared_ptr<ItagsProblemInputs>>();
        mocks::MockDitags ditags(problem_inputs);

        SearchResults<DynIncrementalTaskAllocationNode> results = ditags.search();

        std::cout << results.goal()->allocation() << std::endl;
        std::cout << results.goal()->m_schedule->makespan() << std::endl;
        ditags.writeSolutionToFile("itags_test_output.json", results.goal());
    }

    TEST(DItags, TestSurvivorHard)
    {
        std::ifstream fin("./data/task_allocation/itags_problem_inputs/survivor_problem0.json");
        nlohmann::json j;
        fin >> j;
        std::shared_ptr<const ItagsProblemInputs> problem_inputs = j.get<std::shared_ptr<ItagsProblemInputs>>();
        mocks::MockDitags ditags(problem_inputs);

        SearchResults<DynIncrementalTaskAllocationNode> results = ditags.search();

        std::cout << results.goal()->allocation() << std::endl;
        ditags.writeSolutionToFile("itags_test_output.json", results.goal());
    }

}  // namespace grstapse::unittests