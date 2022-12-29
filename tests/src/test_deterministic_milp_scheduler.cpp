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
#ifndef NO_MILP

// Global
#    include <fstream>
// External
#    include <fmt/format.h>
#    include <gtest/gtest.h>
// Project
#    include <grstapse/common/utilities/constants.hpp>
#    include <grstapse/common/utilities/custom_json_conversions.hpp>
#    include <grstapse/common/utilities/time_keeper.hpp>
#    include <grstapse/geometric_planning/ompl/ompl_environment.hpp>
#    include <grstapse/scheduling/milp/deterministic/deterministic_milp_scheduler.hpp>
#    include <grstapse/scheduling/milp/deterministic/deterministic_schedule.hpp>
#    include <grstapse/task.hpp>
// Local
#    include "mock_deterministic_milp_scheduler.hpp"
#    include "mock_normalized_schedule_quality.hpp"
#    include "scheduling_setup.hpp"

namespace grstapse::unittests
{
    // region DeterministicMilpScheduler::init
    /**!
     * Tests that DeterministicMilpScheduler::init correctly handles different numbers of tasks
     */
    TEST(DeterministicMilpScheduler, init_NumberOfTasks_NoAllocation)
    {
        auto run_test = [](const unsigned int num_tasks, PlanOption plan_option)
        {
            auto schedule_problem_inputs = createSchedulerProblemInputs(plan_option, AllocationOption::e_none, true);
            mocks::MockDeterministicMilpScheduler scheduler(schedule_problem_inputs);  // Runs init

            const std::vector<float>& durations = scheduler.taskDurations();
            ASSERT_EQ(durations.size(), num_tasks);

            const std::vector<TaskTransitionInfo>& initial_transition_info = scheduler.initialTransitionInfo();
            ASSERT_EQ(initial_transition_info.size(), num_tasks);

            const std::vector<std::vector<TaskTransitionInfo>>& transition_info = scheduler.transitionInfo();
            ASSERT_EQ(transition_info.size(), num_tasks);
            for(const std::vector<TaskTransitionInfo>& inner: transition_info)
            {
                ASSERT_EQ(inner.size(), num_tasks);
            }
        };

        run_test(3, PlanOption::e_total_order);
        run_test(3, PlanOption::e_branch);
        run_test(4, PlanOption::e_diamond);
        run_test(4, PlanOption::e_parallel);
        run_test(7, PlanOption::e_complex);
    }

    TEST(DeterministicMilpScheduler, init_Allocation)
    {
        auto run_test = [](AllocationOption allocation_option)
        {
            auto scheduler_problem_inputs =
                createSchedulerProblemInputs(PlanOption::e_total_order, allocation_option, true);
            mocks::MockDeterministicMilpScheduler scheduler(scheduler_problem_inputs);

            const Eigen::MatrixXf& allocation = scheduler.allocation();
            const unsigned int num_robots     = allocation.cols();

            const std::vector<TaskTransitionInfo>& initial_transition_info = scheduler.initialTransitionInfo();
            const unsigned int num_tasks                                   = initial_transition_info.size();

            for(unsigned int task_nr = 0; task_nr < num_tasks; ++task_nr)
            {
                unsigned int correct = 0;
                for(unsigned int robot_nr = 0; robot_nr < num_robots; ++robot_nr)
                {
                    if(allocation(task_nr, robot_nr))
                    {
                        ++correct;
                    }
                }

                ASSERT_EQ(initial_transition_info[task_nr].size(), correct);
            }

            const std::vector<std::vector<TaskTransitionInfo>>& transition_info = scheduler.transitionInfo();
            for(unsigned int task_i = 0; task_i < num_tasks; ++task_i)
            {
                for(unsigned int task_j = 0; task_j < num_tasks; ++task_j)
                {
                    if(task_i == task_j)
                    {
                        continue;
                    }

                    unsigned int correct = 0;
                    for(unsigned int robot_nr = 0; robot_nr < num_robots; ++robot_nr)
                    {
                        if(allocation(task_i, robot_nr) && allocation(task_j, robot_nr))
                        {
                            ++correct;
                        }
                    }
                    ASSERT_EQ(transition_info[task_i][task_j].size(), correct);
                }
            }
        };

        run_test(AllocationOption::e_identity);
        run_test(AllocationOption::e_multi_robot_task);
        run_test(AllocationOption::e_multi_task_robot);
    }
    // endregion

    // region DeterministicMilpScheduler::createTaskDurationsFirstIteration
    /**!
     * Tests that DeterministicMilpScheduler::createTaskDurationsFirstIteration correctly handles different numbers of
     * tasks
     */
    TEST(DeterministicMilpScheduler, createTaskDurationsFirstIteration_SizeOfTaskTimepoints)
    {
        GRBEnv environment(true);
        environment.set(GRB_IntParam_LogToConsole, 0);
        environment.set(GRB_DoubleParam_TimeLimit, 1.0f);
        environment.start();

        auto run_test = [&environment](const std::string& identifier, PlanOption plan_option)
        {
            auto scheduler_problem_inputs = createSchedulerProblemInputs(plan_option, AllocationOption::e_none, true);
            mocks::MockDeterministicMilpScheduler scheduler(scheduler_problem_inputs);
            GRBModel model(environment);
            scheduler.createTaskDurationsFirstIteration(model);
            const std::vector<float>& durations             = scheduler.taskDurations();
            const std::vector<TaskVariableInfo>& timepoints = scheduler.taskTimepoints();
            ASSERT_EQ(timepoints.size(), durations.size())
                << fmt::format("{0:s}: Incorrect number of tasks", identifier);
        };

        run_test("TO", PlanOption::e_total_order);
        run_test("Diamond", PlanOption::e_diamond);
        run_test("Complex", PlanOption::e_complex);
    }

    /**!
     * Tests that DeterministicMilpScheduler::createTaskDurationsFirstIteration correctly handles no allocations
     * (effectively STN)
     */
    TEST(DeterministicMilpScheduler, createTaskDurationsFirstIteration_STN)
    {
        auto run_test = [](PlanOption plan_option)
        {
            auto scheduler_problem_inputs = createSchedulerProblemInputs(plan_option, AllocationOption::e_none, true);
            mocks::MockDeterministicMilpScheduler scheduler(scheduler_problem_inputs);

            GRBEnv environment(true);
            environment.set(GRB_IntParam_LogToConsole, 0);
            environment.set(GRB_DoubleParam_TimeLimit, 1.0f);
            environment.start();
            GRBModel model(environment);
            scheduler.createTaskDurationsFirstIteration(model);

            const std::vector<float>& durations                  = scheduler.taskDurations();
            const std::vector<std::shared_ptr<const Task>> tasks = scheduler.tasks();
            for(unsigned int i = 0, end = durations.size(); i < end; ++i)
            {
                ASSERT_FLOAT_EQ(durations[i], tasks[i]->staticDuration());
            }
        };

        run_test(PlanOption::e_total_order);
        run_test(PlanOption::e_branch);
        run_test(PlanOption::e_diamond);
        run_test(PlanOption::e_parallel);
        run_test(PlanOption::e_complex);
    }

    /**!
     * Tests that DeterministicMilpScheduler::createTaskDurationsFirstIteration correctly handles homogeneous robots
     */
    TEST(DeterministicMilpScheduler, createTaskDurationsFirstIteration_HomogeneousRobots)
    {
        auto run_test =
            [](PlanOption plan_option, AllocationOption allocation_option, const std::vector<float>& correct_durations)
        {
            auto scheduler_problem_inputs = createSchedulerProblemInputs(plan_option, allocation_option, true);
            mocks::MockDeterministicMilpScheduler scheduler(scheduler_problem_inputs);

            GRBEnv environment(true);
            environment.set(GRB_IntParam_LogToConsole, 0);
            environment.set(GRB_DoubleParam_TimeLimit, 1.0f);
            environment.start();
            GRBModel model(environment);
            const bool success = scheduler.createTaskDurationsFirstIteration(model);
            ASSERT_TRUE(success);

            const std::vector<float>& durations = scheduler.taskDurations();
            for(unsigned int i = 0, end = durations.size(); i < end; ++i)
            {
                ASSERT_FLOAT_EQ(durations[i], correct_durations[i]);
            }
        };

        run_test(PlanOption::e_total_order, AllocationOption::e_identity, {1.0f, 7.0f, 16.0f});
        run_test(PlanOption::e_branch, AllocationOption::e_identity, {1.0f, 7.0f, 16.0f});
        run_test(PlanOption::e_diamond, AllocationOption::e_identity, {1.0f, 7.0f, 16.0f, 2.0f});
        run_test(PlanOption::e_parallel, AllocationOption::e_identity, {1.0f, 7.0f, 16.0f, 2.0f});
        run_test(PlanOption::e_complex,
                 AllocationOption::e_identity,
                 {1.0f, 7.0f, 16.0f, 2.0f, 8.65685424949f, 5.7201895692f, 17.2705098312f});
        run_test(PlanOption::e_complex,
                 AllocationOption::e_multi_robot_task,
                 {1.0f, 7.0f, 16.0f, 2.0f, 8.65685424949f, 5.7201895692f, 17.2705098312f});
        run_test(PlanOption::e_complex,
                 AllocationOption::e_multi_task_robot,
                 {1.0f, 7.0f, 16.0f, 2.0f, 8.65685424949f, 5.7201895692f, 17.2705098312f});
    }

    /**!
     * Tests that DeterministicMilpScheduler::createTaskDurationsFirstIteration correctly handles heterogeneous robots
     */
    TEST(DeterministicMilpScheduler, createTaskDurationsFirstIteration_HeterogeneousRobots)
    {
        auto run_test = [](const std::string& identifier,
                           PlanOption plan_option,
                           AllocationOption allocation_option,
                           const std::vector<float>& correct_durations)
        {
            auto scheduler_problem_inputs = createSchedulerProblemInputs(plan_option, allocation_option, false);
            mocks::MockDeterministicMilpScheduler scheduler(scheduler_problem_inputs);

            GRBEnv environment(true);
            environment.set(GRB_IntParam_LogToConsole, 0);
            environment.set(GRB_DoubleParam_TimeLimit, 1.0f);
            environment.start();
            GRBModel model(environment);
            const bool success = scheduler.createTaskDurationsFirstIteration(model);
            ASSERT_TRUE(success) << fmt::format("{0:s}: createTaskDurationsFirstIteration failed", identifier);

            const std::vector<float>& durations = scheduler.taskDurations();
            ASSERT_EQ(durations.size(), correct_durations.size())
                << fmt::format("{0:s}: Incorrect number of task durations", identifier);
            for(unsigned int i = 0, end = durations.size(); i < end; ++i)
            {
                ASSERT_FLOAT_EQ(durations[i], correct_durations[i])
                    << fmt::format("{0:s}: Incorrect duration for task {1:d} (true: {2:f}; computed: {3:f})",
                                   identifier,
                                   i,
                                   correct_durations[i],
                                   durations[i]);
            }
        };

        run_test("TO", PlanOption::e_total_order, AllocationOption::e_identity, {1.0f, 7.0f, 13.5});
        run_test("Branch", PlanOption::e_branch, AllocationOption::e_identity, {1, 7.0f, 13.5});
        run_test("Diamond", PlanOption::e_diamond, AllocationOption::e_identity, {1, 7.0f, 13.5, 2.0f});
        run_test("Parallel", PlanOption::e_parallel, AllocationOption::e_identity, {1, 7.0f, 13.5, 2.0f});
        run_test("Complex Identity",
                 PlanOption::e_complex,
                 AllocationOption::e_identity,
                 {1, 7.0f, 13.5, 2.0f, 7.71404520791f, 5.7201895692f, 14.4754248594f});
        run_test("Complex MT",
                 PlanOption::e_complex,
                 AllocationOption::e_multi_robot_task,
                 {1, 7.0f, 13.5, 2.0f, 7.71404520791f, 5.7201895692f, 14.4754248594f});
        run_test("Complex MR",
                 PlanOption::e_complex,
                 AllocationOption::e_multi_task_robot,
                 {1, 7.0f, 13.5, 2.0f, 7.71404520791f, 5.7201895692f, 14.4754248594f});
    }
    // endregion

    // region DeterministicMilpScheduler::createTaskDurationsOtherIterations
    /**!
     * Tests that DeterministicMilpScheduler::createTaskDurationsOtherIterations is correct
     */
    TEST(DeterministicMilpScheduler, createTaskDurationsOtherOtherIterations)
    {
        auto scheduler_problem_inputs =
            createSchedulerProblemInputs(PlanOption::e_complex, AllocationOption::e_multi_task_robot, false);
        mocks::MockDeterministicMilpScheduler scheduler(scheduler_problem_inputs);

        GRBEnv environment(true);
        environment.set(GRB_IntParam_LogToConsole, 0);
        environment.set(GRB_DoubleParam_TimeLimit, 1.0f);
        environment.start();
        GRBModel model(environment);
        const bool success = scheduler.createTaskDurationsFirstIteration(model);
        ASSERT_TRUE(success);
        const bool second_success = scheduler.createTaskDurationsOtherIterations(model);
        ASSERT_TRUE(second_success);
    }
    // endregion

    // region DeterministicMilpScheduler::createPrecedenceConstraints
    /**!
     * Tests that DeterministicMilpScheduler::createPrecedenceConstraints creates the constraints correctly
     *
     * \note Requires DeterministicMilpScheduler::Objective & MilpScheduler::createTaskDuration to be correct
     */
    TEST(DeterministicMilpScheduler, createPrecedenceConstraints_makespan)
    {
        GRBEnv environment(true);
        environment.set(GRB_IntParam_LogToConsole, 0);
        environment.set(GRB_DoubleParam_TimeLimit, 1.0f);
        environment.start();

        auto run_test = [&environment](const std::string& identifier,
                                       PlanOption plan_option,
                                       bool homogeneous,
                                       const std::vector<float>& true_starts,
                                       const std::vector<float>& true_finishes,
                                       const float true_makespan)
        {
            auto scheduler_problem_inputs =
                createSchedulerProblemInputs(plan_option, AllocationOption::e_identity, homogeneous);
            mocks::MockDeterministicMilpScheduler scheduler(scheduler_problem_inputs);
            GRBModel model(environment);
            scheduler.createTaskDurations(model);
            scheduler.createPrecedenceConstraints(model);
            scheduler.createObjective(model);
            model.optimize();
            const int status = model.get(GRB_IntAttr_Status);
            ASSERT_EQ(status, GRB_OPTIMAL);

            const std::vector<TaskVariableInfo>& task_timepoints = scheduler.taskTimepoints();

            for(unsigned int i = 0, end = task_timepoints.size(); i < end; ++i)
            {
                const float task_start = task_timepoints[i].start.get(GRB_DoubleAttr_X);
                ASSERT_FLOAT_EQ(task_start, true_starts[i])
                    << fmt::format("{0:s}: Task {1:d} start point is incorrect (true: {2:f}; computed: {3:f})",
                                   identifier,
                                   i,
                                   true_starts[i],
                                   task_start);
                const float task_finish = task_timepoints[i].finish.get(GRB_DoubleAttr_X);
                ASSERT_FLOAT_EQ(task_finish, true_finishes[i])
                    << fmt::format("{0:s}: Task {1:d} finish point is incorrect (true: {2:f}; computed: {3:f})",
                                   identifier,
                                   i,
                                   true_finishes[i],
                                   task_finish);
            }

            const double makespan = model.getVarByName(constants::k_makespan).get(GRB_DoubleAttr_X);
            ASSERT_FLOAT_EQ(makespan, true_makespan)
                << fmt::format("{0:s}: Makespan is incorrect (true: {1:f}; computed: {2:f})",
                               identifier,
                               true_makespan,
                               makespan);
        };

        run_test("TO", PlanOption::e_total_order, true, {0.0f, 1.0f, 8.0f}, {1.0f, 8.0f, 24.0f}, 24.0f);
        run_test("Branch", PlanOption::e_branch, true, {0.0f, 1.0f, 1.0f}, {1.0f, 8.0f, 17.0f}, 17.0f);
        run_test("Diamond", PlanOption::e_diamond, true, {0.0f, 1.0f, 1.0f, 17.0f}, {1.0f, 8.0f, 17.0f, 19.0f}, 19.0f);
        run_test("Parallel",
                 PlanOption::e_parallel,
                 true,
                 {0.0f, 1.0f, 0.0f, 16.0f},
                 {1.0f, 8.0f, 16.0f, 18.0f},
                 18.0f);
        run_test("Complex Homo",
                 PlanOption::e_complex,
                 true,
                 {0.0f, 1.0f, 5.7201895692f, 21.7201895692f, 23.7201895692f, 0.0f, 5.7201895692f},
                 {1.0f, 8.0f, 21.7201895692f, 23.7201895692f, 32.3770438187f, 5.7201895692f, 22.9906994004f},
                 32.3770438187f);
        run_test("Complex Hetero",
                 PlanOption::e_complex,
                 false,
                 {0.0f, 1.0f, 5.7201895692f, 19.220190f, 21.220190f, 0.0f, 5.7201895692f},
                 {1.0f, 8.0f, 19.220190f, 21.220190f, 28.934235f, 5.7201895692f, 20.195614f},
                 28.934235f);
    }
    // endregion

    // region DeterministicMilpScheduler::createMutexConstraintsFirstIteration
    /**!
     * Tests that DeterministicMilpScheduler::createMutexConstraintsFirstIteration correctly reduces the mutex
     * constraint set
     *
     * \note Depends on DeterministicMilpScheduler::createTaskDurations working
     */
    TEST(DeterministicMilpScheduler, createMutexConstraintsFirstIteration)
    {
        GRBEnv environment(true);
        environment.set(GRB_IntParam_LogToConsole, 0);
        environment.set(GRB_DoubleParam_TimeLimit, 1.0f);
        environment.start();

        auto run_test = [&environment](const std::string& identifier,
                                       PlanOption plan_option,
                                       AllocationOption allocation_option,
                                       bool homogeneous,
                                       const std::set<std::pair<unsigned int, unsigned int>>& correct_reduced_mutexes)
        {
            auto scheduler_problem_inputs = createSchedulerProblemInputs(plan_option, allocation_option, homogeneous);
            mocks::MockDeterministicMilpScheduler scheduler(scheduler_problem_inputs);

            GRBModel model(environment);
            ASSERT_TRUE(scheduler.createTaskDurations(model));
            ASSERT_TRUE(scheduler.createMutexConstraintsFirstIteration(model))
                << fmt::format("{0:s}: createMutexConstraintsFirstIteration failed", identifier);

            const robin_hood::unordered_map<std::string, MutexConstraintInfo>& reduced_mutex_constraints =
                scheduler.reducedMutexConstraints();
            ASSERT_EQ(correct_reduced_mutexes.size(), reduced_mutex_constraints.size())
                << fmt::format("{0:s}: Incorrect number of mutex constraints (true: {1:d}; computed: {2:d})",
                               identifier,
                               correct_reduced_mutexes.size(),
                               reduced_mutex_constraints.size());
            for(const std::pair<unsigned int, unsigned int>& mutex_constraint: correct_reduced_mutexes)
            {
                const std::string name =
                    fmt::format("p_({0:d},{1:d})", mutex_constraint.first, mutex_constraint.second);
                ASSERT_TRUE(reduced_mutex_constraints.contains(name))
                    << fmt::format("{0:s}: Reduced mutex constraints missing a mutex constraint ({1:d} <-> {2:d})",
                                   identifier,
                                   mutex_constraint.first,
                                   mutex_constraint.second);
            }
        };

        run_test("TO", PlanOption::e_total_order, AllocationOption::e_multi_task_robot, true, {});
        run_test("Branch", PlanOption::e_branch, AllocationOption::e_multi_task_robot, true, {});
        run_test("Diamond", PlanOption::e_diamond, AllocationOption::e_multi_task_robot, true, {});
        run_test("Parallel", PlanOption::e_parallel, AllocationOption::e_multi_task_robot, true, {{0, 3}});
        run_test("Complex", PlanOption::e_complex, AllocationOption::e_complex, true, {});
        run_test("Complex2", PlanOption::e_complex, AllocationOption::e_complex2, true, {{0, 6}, {2, 6}, {1, 5}});
    }
    // endregion

    // region DeterministicMilpScheduler::createMutexConstraintsOtherIterations
    /**!
     * Tests that DeterministicMilpScheduler::createMutexConstraintsOtherIterations is correct
     */
    TEST(DeterministicMilpScheduler, createMutexConstraintsOtherIterations)
    {
        GRBEnv environment(true);
        environment.set(GRB_IntParam_LogToConsole, 0);
        environment.set(GRB_DoubleParam_TimeLimit, 1.0f);
        environment.start();

        auto run_test = [&environment](const std::string& identifier,
                                       PlanOption plan_option,
                                       AllocationOption allocation_option,
                                       bool homogeneous,
                                       const std::set<std::pair<unsigned int, unsigned int>>& correct_reduced_mutexes)
        {
            auto scheduler_problem_inputs = createSchedulerProblemInputs(plan_option, allocation_option, homogeneous);
            mocks::MockDeterministicMilpScheduler scheduler(scheduler_problem_inputs);

            GRBModel model(environment);
            ASSERT_TRUE(scheduler.createTaskDurations(model));
            ASSERT_TRUE(scheduler.createMutexConstraintsFirstIteration(model))
                << fmt::format("{0:s}: createMutexConstraintsFirstIteration failed", identifier);
            ASSERT_TRUE(scheduler.createMutexConstraintsOtherIterations(model))
                << fmt::format("{0:s}: createMutexConstraintsOtherIterations failed", identifier);

            const robin_hood::unordered_map<std::string, MutexConstraintInfo>& reduced_mutex_constraints =
                scheduler.reducedMutexConstraints();
            ASSERT_EQ(correct_reduced_mutexes.size(), reduced_mutex_constraints.size())
                << fmt::format("{0:s}: Incorrect number of mutex constraints (true: {1:d}; computed: {2:d})",
                               identifier,
                               correct_reduced_mutexes.size(),
                               reduced_mutex_constraints.size());
            for(const std::pair<unsigned int, unsigned int>& mutex_constraint: correct_reduced_mutexes)
            {
                const std::string name =
                    fmt::format("p_({0:d},{1:d})", mutex_constraint.first, mutex_constraint.second);
                ASSERT_TRUE(reduced_mutex_constraints.contains(name))
                    << fmt::format("{0:s}: Reduced mutex constraints missing a mutex constraint ({1:d} <-> {2:d})",
                                   identifier,
                                   mutex_constraint.first,
                                   mutex_constraint.second);
            }
        };

        run_test("TO", PlanOption::e_total_order, AllocationOption::e_multi_task_robot, true, {});
        run_test("Branch", PlanOption::e_branch, AllocationOption::e_multi_task_robot, true, {});
        run_test("Diamond", PlanOption::e_diamond, AllocationOption::e_multi_task_robot, true, {});
        run_test("Parallel", PlanOption::e_parallel, AllocationOption::e_multi_task_robot, true, {{0, 3}});
        run_test("Complex", PlanOption::e_complex, AllocationOption::e_complex, true, {});
        run_test("Complex2", PlanOption::e_complex, AllocationOption::e_complex2, true, {{0, 6}, {2, 6}, {1, 5}});
    }
    // endregion

    // region DeterministicMilpScheduler::createInitialTransitions
    /**!
     * Tests that DeterministicMilpScheduler::createInitialTransitions is correct
     */
    TEST(DeterministicMilpScheduler, createInitialTransitions)
    {
        GRBEnv environment(true);
        environment.set(GRB_IntParam_LogToConsole, 0);
        environment.set(GRB_DoubleParam_TimeLimit, 1.0f);
        environment.start();

        auto run_test = [&environment](const std::string& identifier,
                                       PlanOption plan_option,
                                       AllocationOption allocation_option,
                                       bool homogeneous)
        {
            auto scheduler_problem_inputs = createSchedulerProblemInputs(plan_option, allocation_option, homogeneous);
            mocks::MockDeterministicMilpScheduler scheduler(scheduler_problem_inputs);

            GRBModel model(environment);
            ASSERT_TRUE(scheduler.createTaskDurations(model))
                << fmt::format("{0:s}: createTaskDurations failed", identifier);
            ASSERT_TRUE(scheduler.createInitialTransitions(model))
                << fmt::format("{0:s}: createInitialTransitions failed", identifier);
        };

        run_test("TO", PlanOption::e_total_order, AllocationOption::e_multi_task_robot, true);
        run_test("Branch", PlanOption::e_branch, AllocationOption::e_multi_task_robot, true);
        run_test("Diamond", PlanOption::e_diamond, AllocationOption::e_multi_task_robot, true);
        run_test("Parallel", PlanOption::e_parallel, AllocationOption::e_multi_task_robot, true);
        run_test("Complex", PlanOption::e_complex, AllocationOption::e_complex, true);
        run_test("Complex2", PlanOption::e_complex, AllocationOption::e_complex2, true);
    }
    // endregion

    // region DeterministicMilpScheduler::createMakespan
    /**!
     * Tests that DeterministicMilpScheduler::Objective is correct
     */
    //    TEST(DeterministicMilpScheduler, Objective)
    //    {
    //        throw std::exception();
    //    }
    // endregion

    // region DeterministicMilpScheduler::computeInitialTransitionHeuristicDuration
    /**!
     * Tests that DeterministicMilpScheduler::computeInitialTransitionHeuristicDuration is correct
     */
    TEST(DeterministicMilpScheduler, computeInitialTransitionHeuristicDuration)
    {
        GRBEnv environment(true);
        environment.set(GRB_IntParam_LogToConsole, 0);
        environment.set(GRB_DoubleParam_TimeLimit, 1.0f);
        environment.start();

        auto run_test = [&environment](const std::string& identifier,
                                       PlanOption plan_option,
                                       AllocationOption allocation_option,
                                       bool homogeneous,
                                       const std::vector<std::vector<float>>& correct_transitions)
        {
            auto scheduler_problem_inputs = createSchedulerProblemInputs(plan_option, allocation_option, homogeneous);
            mocks::MockDeterministicMilpScheduler scheduler(scheduler_problem_inputs);

            GRBModel model(environment);
            ASSERT_TRUE(scheduler.createTaskDurations(model))
                << fmt::format("{0:s}: createTaskDurations failed", identifier);
            scheduler.computeInitialTransitionHeuristicDurations();

            const std::vector<TaskTransitionInfo>& initial_transition_info = scheduler.initialTransitionInfo();
            ASSERT_EQ(correct_transitions.size(), initial_transition_info.size())
                << fmt::format("{0:s}: Incorrect number of tasks", identifier);
            for(unsigned int i = 0, endi = correct_transitions.size(); i < endi; ++i)
            {
                ASSERT_EQ(correct_transitions[i].size(), initial_transition_info[i].size())
                    << fmt::format("{0:s}: Incorrect number of robots on task {1:d}", identifier, i);
                for(unsigned int j = 0, endj = correct_transitions[i].size(); j < endj; ++j)
                {
                    ASSERT_FLOAT_EQ(initial_transition_info[i][j].duration, correct_transitions[i][j])
                        << fmt::format("{0:s}: Incorrect initial transition duration for task {1:d} robot {2:d} (true: "
                                       "{3:f}; computed: {4:f})",
                                       identifier,
                                       i,
                                       initial_transition_info[i][j].robot_nr,
                                       correct_transitions[i][j],
                                       initial_transition_info[i][j].duration);
                }
            }
        };

        run_test("TO", PlanOption::e_total_order, AllocationOption::e_identity, true, {{5.0f}, {5.0f}, {5.0f}});
        run_test("Complex",
                 PlanOption::e_complex,
                 AllocationOption::e_identity,
                 true,
                 {{5.0f}, {5.0f}, {5.0f}, {15.0f}, {14.577379737113251f}, {16.387800340497193f}, {32.01562118716424}});
        run_test("Complex",
                 PlanOption::e_complex,
                 AllocationOption::e_complex,
                 true,
                 {{5.0f},
                  {7.0710678118654755f},
                  {7.0710678118654755f},
                  {21.213203435596423f},
                  {17.67766952966369f},
                  {20.113676938839404f, 17.191858538273284f},
                  {47.169905660283014f}});
        run_test("Complex",
                 PlanOption::e_complex,
                 AllocationOption::e_complex,
                 false,
                 {{4.166666666666667f},
                  {5.892556509887896f},
                  {7.0710678118654755f},
                  {17.67766952966369f},
                  {14.73139127471974f},
                  {20.113676938839404f, 14.326548781894404f},
                  {39.30825471690252f}});
    }
    // endregion

    // region DeterministicMilpScheduler::computeTransitionHeuristicDuration
    /**!
     * Tests that DeterministicMilpScheduler::computeTransitionHeuristicDuration is correct
     */
    TEST(DeterministicMilpScheduler, computeTransitionHeuristicDuration)
    {
        GRBEnv environment(true);
        environment.set(GRB_IntParam_LogToConsole, 0);
        environment.set(GRB_DoubleParam_TimeLimit, 1.0f);
        environment.start();

        auto run_test = [&environment](const std::string& identifier,
                                       PlanOption plan_option,
                                       AllocationOption allocation_option,
                                       bool homogeneous,
                                       const std::vector<std::vector<std::vector<float>>>& correct_transitions)
        {
            auto scheduler_problem_inputs = createSchedulerProblemInputs(plan_option, allocation_option, homogeneous);
            mocks::MockDeterministicMilpScheduler scheduler(scheduler_problem_inputs);

            GRBModel model(environment);
            ASSERT_TRUE(scheduler.createTaskDurations(model))
                << fmt::format("{0:s}: createTaskDurations failed", identifier);
            scheduler.computeTransitionHeuristicDurations();
            const std::vector<std::vector<TaskTransitionInfo>>& transition_durations = scheduler.transitionInfo();

            ASSERT_EQ(transition_durations.size(), correct_transitions.size())
                << fmt::format("{0:s}: Incorrect number of tasks", identifier);
            for(unsigned int i = 0, endi = transition_durations.size(); i < endi; ++i)
            {
                ASSERT_EQ(transition_durations[i].size(), correct_transitions[i].size())
                    << fmt::format("{0:s}: Incorrect number of tasks", identifier);
                for(unsigned int j = 0, endj = transition_durations[i].size(); j < endj; ++j)
                {
                    if(i == j)
                    {
                        ASSERT_TRUE(transition_durations[i][j].empty())
                            << fmt::format("{0:s}: Self transition should have no robots (task {1:d})", identifier, i);
                        continue;
                    }
                    ASSERT_EQ(transition_durations[i][j].size(), correct_transitions[i][j].size())
                        << fmt::format("{0:s}: Incorrect number of robots on transition from task {1:d} to task {2:d}",
                                       identifier,
                                       i,
                                       j);

                    for(unsigned int k = 0, endk = transition_durations[i][j].size(); k < endk; ++k)
                    {
                        ASSERT_NEAR(transition_durations[i][j][k].duration, correct_transitions[i][j][k], 1e-4)
                            << fmt::format("{0:s}: Incorrect transition for robot {1:d} from task {2:d} to {3:d} "
                                           "(true: {4:f}; computed: {5:f})",
                                           identifier,
                                           transition_durations[i][j][k].robot_nr,
                                           i,
                                           j,
                                           correct_transitions[i][j][k],
                                           transition_durations[i][j][k].duration);
                    }
                }
            }
        };

        run_test("TO",
                 PlanOption::e_total_order,
                 AllocationOption::e_identity,
                 true,
                 {{{}, {}, {}}, {{}, {}, {}}, {{}, {}, {}}});
        run_test("Complex-Homo",
                 PlanOption::e_complex,
                 AllocationOption::e_complex,
                 true,
                 {
                     {
                         {},
                         {
                             5.0f,
                         },
                         {},
                         {
                             18.027756377319946f,
                         },
                         {
                             14.577379737113251f,
                         },
                         {},
                         {},
                     },
                     {
                         {
                             7.071068f,
                         },
                         {},
                         {},
                         {
                             11.180340f,
                         },
                         {
                             7.905694f,
                         },
                         {},
                         {},
                     },
                     {
                         {},
                         {},
                         {},
                         {},
                         {},
                         {9.7754802703857422f},
                         {},
                     },
                     {
                         {
                             18.027756377319946f,
                         },
                         {
                             14.142135623730951f,
                         },
                         {},
                         {},
                         {
                             3.5355339050292969f,
                         },
                         {},
                         {},
                     },
                     {
                         {
                             9.1923885345458984f,
                         },
                         {
                             4.9497480392456055f,
                         },
                         {},
                         {
                             9.192387580871582f,
                         },
                         {},
                         {},
                         {},
                     },
                     {
                         {},
                         {},
                         {9.0138778686523438f},
                         {},
                         {},
                         {},
                         {37.165172576904297f},
                     },
                     {
                         {},
                         {},
                         {},
                         {},
                         {},
                         {16.787197113037109f},
                         {},
                     },
                 });
        run_test("Complex-Hetero",
                 PlanOption::e_complex,
                 AllocationOption::e_complex,
                 false,
                 {
                     {
                         {},
                         {
                             4.166666666666667f,
                         },
                         {},
                         {
                             15.023130314433288f,
                         },
                         {
                             12.147816447594376f,
                         },
                         {},
                         {},
                     },
                     {
                         {
                             5.8925566673278809f,
                         },
                         {},
                         {},
                         {
                             9.3169498443603516f,
                         },
                         {
                             6.58807849884033f,
                         },
                         {},
                         {},
                     },
                     {
                         {},
                         {},
                         {},
                         {},
                         {},
                         {
                             9.7754802703857422f,
                         },
                         {},
                     },
                     {
                         {
                             15.023130314433288f,
                         },
                         {
                             11.785113019775793f,
                         },
                         {},
                         {},
                         {
                             2.946278254943948f,
                         },
                         {},
                         {},
                     },
                     {
                         {
                             7.6603240966796875f,
                         },
                         {
                             4.1247901916503906f,
                         },
                         {},
                         {
                             7.6603236198425293f,
                         },
                         {},
                         {},
                         {},
                     },
                     {
                         {},
                         {},
                         {
                             9.0138778686523438f,
                         },
                         {},
                         {},
                         {},
                         {30.970977783203125f},
                     },
                     {
                         {},
                         {},
                         {},
                         {},
                         {},
                         {13.989331245422363f},
                         {},
                     },
                 });
    }
    // endregion

    // region Optimization
    /**!
     * Test that a single iteration of the scheduler works
     *
     * \note As this is the first iteration it uses euclidean distance heuristic
     */
    TEST(DeterministicMilpScheduler, SingleIteration)
    {
        GRBEnv environment(true);
        environment.set(GRB_IntParam_LogToConsole, 0);
        environment.set(GRB_DoubleParam_TimeLimit, 1.0f);
        environment.start();

        auto run_test = [&environment](const std::string& identifier,
                                       const PlanOption plan_option,
                                       const AllocationOption allocation_option,
                                       const bool homogeneous,
                                       const std::vector<std::pair<float, float>>& correct_timepoints,
                                       const float correct_makespan)
        {
            auto scheduler_problem_inputs = createSchedulerProblemInputs(plan_option, allocation_option, homogeneous);
            mocks::MockDeterministicMilpScheduler scheduler(scheduler_problem_inputs);
            // Would be in solve
            scheduler.computeInitialTransitionHeuristicDurations();
            scheduler.computeTransitionHeuristicDurations();

            GRBModel model(environment);
            ASSERT_TRUE(scheduler.createModel(model)) << fmt::format("{0:s}: Failed to build model", identifier);

            // Optimize model
            model.optimize();

            // Check status
            ASSERT_EQ(model.get(GRB_IntAttr_Status), GRB_OPTIMAL)
                << fmt::format("{0:s}: Model failed to optimize", identifier);

            auto schedule = std::dynamic_pointer_cast<const DeterministicSchedule>(scheduler.createSchedule(model));
            ASSERT_TRUE(schedule);
            ASSERT_NEAR(schedule->makespan(), correct_makespan, 1e-4)
                << fmt::format("{0:s}: Incorrect makespan (true: {1:f}; computed: {2:f})",
                               identifier,
                               correct_makespan,
                               schedule->makespan());

            auto timepoints = schedule->timepoints();
            for(unsigned int i = 0, end = timepoints.size(); i < end; ++i)
            {
                ASSERT_NEAR(timepoints[i].first, correct_timepoints[i].first, 1e-4)
                    << fmt::format("{0:s}: Incorrect start timepoint for task {1:d} (true: {2:f}; computed: {3:f})",
                                   identifier,
                                   i,
                                   correct_timepoints[i].first,
                                   timepoints[i].first);
                ASSERT_NEAR(timepoints[i].second, correct_timepoints[i].second, 1e-4)
                    << fmt::format("{0:s}: Incorrect start timepoint for task {1:d} (true: {2:f}; computed: {3:f})",
                                   identifier,
                                   i,
                                   correct_timepoints[i].second,
                                   timepoints[i].second);
            }
        };

        // First iteration is without transition updates, so "schedule" is based on euclidean distance heuristic
        run_test("TO-I",
                 PlanOption::e_total_order,
                 AllocationOption::e_identity,
                 true,
                 {{5.0f, 6.0f}, {6.0f, 13.0f}, {13.0f, 29.0f}},
                 29.0f);
        run_test("Branch-I",
                 PlanOption::e_branch,
                 AllocationOption::e_identity,
                 true,
                 {{5.0f, 6.0f}, {6.0f, 13.0f}, {6.0f, 22.0f}},
                 22.0f);
        // Robot 1 does tasks 1 and 3 (tests transition)
        run_test("Branch-MR",
                 PlanOption::e_branch,
                 AllocationOption::e_multi_task_robot,
                 true,
                 {{5.0f, 6.0f}, {6.0f, 13.0f}, {16.0f, 32.0f}},
                 32.0f);
        // If this passes, then the first iteration works
        run_test("Complex 2",
                 PlanOption::e_complex,
                 AllocationOption::e_complex2,
                 false,
                 {{4.1667f, 5.1667f},
                  {38.3339f, 45.3339f},
                  {25.8339f, 39.3339f},
                  {56.5142f, 58.5142f},
                  {58.5142f, 66.2283f},
                  {20.1137f, 25.8339f},
                  {72.9266f, 87.4020f}},
                 87.4020f);
    }

    /**!
     * Test that a full run of scheduler works
     */
    TEST(DeterministicMilpScheduler, FullRun)
    {
        GRBEnv environment(true);
        environment.set(GRB_IntParam_LogToConsole, 0);
        environment.set(GRB_DoubleParam_TimeLimit, 1.0f);
        environment.start();

        auto run_test = [&environment](const std::string& identifier,
                                       const PlanOption plan_option,
                                       const AllocationOption allocation_option,
                                       const bool homogeneous,
                                       const std::vector<std::pair<float, float>>& correct_timepoints,
                                       const float correct_makespan)
        {
            auto scheduler_problem_inputs = createSchedulerProblemInputs(plan_option, allocation_option, homogeneous);
            mocks::MockDeterministicMilpScheduler scheduler(scheduler_problem_inputs);

            auto schedule = std::dynamic_pointer_cast<const DeterministicSchedule>(scheduler.solve());
            ASSERT_TRUE(schedule);

            ASSERT_NEAR(schedule->makespan(), correct_makespan, 1e-4)
                << fmt::format("{0:s}: Incorrect makespan (true: {1:f}; computed: {2:f})",
                               identifier,
                               correct_makespan,
                               schedule->makespan());

            const auto& timepoints = schedule->timepoints();
            for(unsigned int i = 0, end = timepoints.size(); i < end; ++i)
            {
                ASSERT_NEAR(timepoints[i].first, correct_timepoints[i].first, 1e-4)
                    << fmt::format("{0:s}: Incorrect start timepoint for task {1:d} (true: {2:f}; computed: {3:f})",
                                   identifier,
                                   i,
                                   correct_timepoints[i].first,
                                   timepoints[i].first);
                ASSERT_NEAR(timepoints[i].second, correct_timepoints[i].second, 1e-4)
                    << fmt::format("{0:s}: Incorrect start timepoint for task {1:d} (true: {2:f}; computed: {3:f})",
                                   identifier,
                                   i,
                                   correct_timepoints[i].second,
                                   timepoints[i].second);
            }
        };

        run_test("TO-I",
                 PlanOption::e_total_order,
                 AllocationOption::e_identity,
                 true,
                 {{5.0f, 6.0f}, {6.0f, 13.0f}, {13.0f, 29.0f}},
                 29.0f);
        run_test("Branch-I",
                 PlanOption::e_branch,
                 AllocationOption::e_identity,
                 true,
                 {{5.0f, 6.0f}, {6.0f, 13.0f}, {6.0f, 22.0f}},
                 22.0f);
        // Robot 1 does tasks 1 and 3 (tests transition)
        run_test("Branch-MR",
                 PlanOption::e_branch,
                 AllocationOption::e_multi_task_robot,
                 true,
                 {{5.0f, 6.0f}, {6.0f, 13.0f}, {16.0f, 32.0f}},
                 32.0f);
        // If this passes, then the scheduler iteration works
        run_test("Complex 2",
                 PlanOption::e_complex,
                 AllocationOption::e_complex2,
                 false,
                 {{4.1667f, 5.1667f},
                  {38.3339f, 45.3339f},
                  {25.8339f, 39.3339f},
                  {56.5142f, 58.5142f},
                  {58.5142f, 66.2283f},
                  {20.1137f, 25.8339f},
                  {72.9266f, 87.4020f}},
                 87.4020f);
    }

    TEST(DeterministicMilpScheduler, GlenDitagsTest)
    {
        std::ifstream in("data/task_allocation/itags_problem_inputs/survivor_problem0.json");
        nlohmann::json j;
        in >> j;
        auto itags_problem_inputs = j.get<std::shared_ptr<ItagsProblemInputs>>();

        Eigen::Matrix<float, 20, 6> allocation;
        allocation << 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f,
            0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f,
            0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f,
            0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f,
            0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f,
            0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f,
            0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f;
        mocks::MockNormalizedScheduleQuality nsq(itags_problem_inputs, 0.0f);
        auto mutex_constraints = nsq.computeMutexConstraints(allocation);

        auto scheduler_problem_inputs =
            std::make_shared<SchedulerProblemInputs>(itags_problem_inputs, allocation, mutex_constraints);
        DeterministicMilpScheduler scheduler(scheduler_problem_inputs);
        auto schedule  = scheduler.solve();
        float mp_time  = TimeKeeper::instance().time(constants::k_motion_planning_time);
        float smp_time = TimeKeeper::instance().time(constants::k_scheduling_time);
        float s_time   = smp_time - mp_time;
        std::cout << fmt::format("MP Time: {0:f}s\nS Time: {1:f}s\nNum MILPs run: {2:d}\n",
                                 mp_time,
                                 s_time,
                                 MilpSchedulerBase::numIterations());
    }
}  // namespace grstapse::unittests
#endif