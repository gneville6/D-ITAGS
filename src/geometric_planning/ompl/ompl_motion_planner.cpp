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
#include "grstapse/geometric_planning/ompl/ompl_motion_planner.hpp"

// External
#include <ompl/base/terminationconditions/CostConvergenceTerminationCondition.h>
#include <ompl/geometric/planners/prm/LazyPRM.h>
#include <ompl/geometric/planners/prm/LazyPRMstar.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <ompl/geometric/planners/rrt/LazyRRT.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/pRRT.h>
// Local
#include "grstapse/common/utilities/error.hpp"
#include "grstapse/common/utilities/logger.hpp"
#include "grstapse/geometric_planning/ompl/ompl_configuration.hpp"
#include "grstapse/geometric_planning/ompl/ompl_environment.hpp"
#include "grstapse/geometric_planning/ompl/ompl_motion_planner_parameters.hpp"

namespace grstapse {
    OmplMotionPlanner::OmplMotionPlanner(OmplMotionPlannerType ompl_motion_planner_type,
                                         const std::shared_ptr<const OmplMotionPlannerParameters> &parameters,
                                         const std::shared_ptr<OmplEnvironment> &environment)
            : MotionPlannerBase(parameters, environment), m_simple_setup(nullptr),
              m_ompl_motion_planner_type(ompl_motion_planner_type) {
        ompl::msg::noOutputHandler();

        // Simple Setup
        auto ompl_environment = std::dynamic_pointer_cast<OmplEnvironment>(m_environment);
        m_simple_setup = std::make_unique<ompl::geometric::SimpleSetup>(ompl_environment->stateSpace());
        m_simple_setup->setStateValidityChecker(ompl_environment);
        std::shared_ptr<ompl::base::Planner> motion_planner;
        switch (ompl_motion_planner_type) {
            case OmplMotionPlannerType::e_prm: {
                motion_planner = std::make_shared<ompl::geometric::PRM>(m_simple_setup->getSpaceInformation());
                break;
            }
            case OmplMotionPlannerType::e_prm_star: {
                motion_planner = std::make_shared<ompl::geometric::PRMstar>(m_simple_setup->getSpaceInformation());
                break;
            }
            case OmplMotionPlannerType::e_lazy_prm: {
                motion_planner = std::make_shared<ompl::geometric::LazyPRM>(m_simple_setup->getSpaceInformation());
                break;
            }
            case OmplMotionPlannerType::e_lazy_prm_star: {
                motion_planner = std::make_shared<ompl::geometric::LazyPRMstar>(m_simple_setup->getSpaceInformation());
                break;
            }
            case OmplMotionPlannerType::e_rrt: {
                motion_planner = std::make_shared<ompl::geometric::RRT>(m_simple_setup->getSpaceInformation());
                break;
            }
            case OmplMotionPlannerType::e_rrt_star: {
                motion_planner = std::make_shared<ompl::geometric::RRTstar>(m_simple_setup->getSpaceInformation());
                break;
            }
            case OmplMotionPlannerType::e_parallel_rrt: {
                motion_planner = std::make_shared<ompl::geometric::pRRT>(m_simple_setup->getSpaceInformation());
                break;
            }
            case OmplMotionPlannerType::e_rrt_connect: {
                motion_planner = std::make_shared<ompl::geometric::RRTConnect>(m_simple_setup->getSpaceInformation());
                break;
            }
            case OmplMotionPlannerType::e_lazy_rrt: {
                motion_planner = std::make_shared<ompl::geometric::LazyRRT>(m_simple_setup->getSpaceInformation());
                break;
            }
            default: {
                throw createLogicError("Unknown motion planner type");
            }
        }
        m_simple_setup->setPlanner(motion_planner);
    }

    const std::shared_ptr<ompl::base::SpaceInformation> &OmplMotionPlanner::spaceInformation() const {
        std::lock_guard<std::mutex> lock(m_mutex);
        return m_simple_setup->getSpaceInformation();
    }

    std::shared_ptr<const MotionPlanningQueryResultBase> OmplMotionPlanner::computeMotionPlan(
            const std::shared_ptr<const Species> &species,
            const std::shared_ptr<const ConfigurationBase> &initial_configuration,
            const std::shared_ptr<const ConfigurationBase> &goal_configuration) {
        std::lock_guard<std::mutex> lock(m_mutex);
        if (!m_simple_setup) {
            throw createLogicError("Motion planning not initialized");
        }

        const auto initial_configuration_ompl =
                std::dynamic_pointer_cast<const OmplConfiguration>(initial_configuration);
        const auto goal_configuration_ompl = std::dynamic_pointer_cast<const OmplConfiguration>(goal_configuration);

        // Clears internal from previous query
        m_simple_setup->getPlanner()->clearQuery();
        m_simple_setup->getProblemDefinition()->clearSolutionPaths();

        // Set start and goal
        ompl::base::ScopedStatePtr scoped_initial_state =
                initial_configuration_ompl->convertToScopedStatePtr(m_simple_setup->getStateSpace());
        if (!scoped_initial_state->satisfiesBounds()) {
            throw createLogicError("Initial state doesn't respect the bounds of the state space");
        }
        m_simple_setup->setStartState(*scoped_initial_state);
        m_simple_setup->setGoal(goal_configuration_ompl->convertToGoalPtr(m_simple_setup->getSpaceInformation()));

        // Set the radius of the robot
        m_environment->lock();
        m_environment->setSpecies(species);
        const auto &ompl_mp_parameters = std::dynamic_pointer_cast<const OmplMotionPlannerParameters>(m_parameters);
        const ompl::base::PlannerStatus status = m_simple_setup->solve(ompl::base::plannerOrTerminationCondition(
                ompl::base::timedPlannerTerminationCondition(ompl_mp_parameters->timeout),
                ompl::base::CostConvergenceTerminationCondition(m_simple_setup->getProblemDefinition(),
                                                                ompl_mp_parameters->solutions_window,
                                                                ompl_mp_parameters->convergence_epislon)));

        if (m_simple_setup->haveSolutionPath()) {
            if (std::dynamic_pointer_cast<const OmplMotionPlannerParameters>(m_parameters)->simplify_path) {
                m_simple_setup->simplifySolution(
                        std::dynamic_pointer_cast<const OmplMotionPlannerParameters>(
                                m_parameters)->simplify_path_timeout);
            }
            // Clear the species from the environment
            m_environment->setSpecies(nullptr);
            m_environment->unlock();
            const ompl::geometric::PathGeometric &path = m_simple_setup->getSolutionPath();
            auto path_ptr = std::make_shared<const ompl::geometric::PathGeometric>(path);
            return std::make_shared<const OmplMotionPlanningQueryResult>(MotionPlannerQueryStatus::e_success, path_ptr);
        } else {
            // Clear the species from the environment
            m_environment->setSpecies(nullptr);
            m_environment->unlock();
            ++s_num_failures;
#if DEBUG
            Logger::debug("Motion planning exceeded the time threshold");
#endif
            return std::make_shared<const OmplMotionPlanningQueryResult>(MotionPlannerQueryStatus::e_timeout, nullptr);
        }
    }
}  // namespace grstapse