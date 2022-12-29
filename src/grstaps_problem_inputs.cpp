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
#include "grstapse/grstaps_problem_inputs.hpp"

// Global
#include <fstream>
// External
#include <fmt/format.h>
// Local
#include "grstapse/common/search/best_first_search_parameters.hpp"
#include "grstapse/common/utilities/constants.hpp"
#include "grstapse/common/utilities/custom_json_conversions.hpp"
#include "grstapse/common/utilities/error.hpp"
#include "grstapse/common/utilities/logger.hpp"
#include "grstapse/geometric_planning/configuration_base.hpp"
#include "grstapse/geometric_planning/ompl/ompl_environment.hpp"
#include "grstapse/geometric_planning/ompl/ompl_motion_planner.hpp"
#include "grstapse/geometric_planning/ompl/ompl_motion_planner_parameters.hpp"
#include "grstapse/geometric_planning/ompl/se2_ompl_configuration.hpp"
#include "grstapse/geometric_planning/ompl/se3_ompl_configuration.hpp"
#include "grstapse/robot.hpp"
#include "grstapse/scheduling/scheduler_parameters.hpp"
#include "grstapse/species.hpp"
#include "grstapse/task.hpp"
#include "grstapse/task_allocation/itags/robot_traits_matrix_reduction.hpp"
#include "grstapse/task_planning/sas/sas_action.hpp"

namespace grstapse {
    GrstapsProblemInputs::GrstapsProblemInputs()
            : m_task_configuration_type(ConfigurationType::e_unknown),
              m_ompl_state_space_type(OmplStateSpaceType::e_unknown) {}


    GrstapsProblemInputs::GrstapsProblemInputs(
            std::shared_ptr<const BestFirstSearchParameters> fcpop_parameters,
            std::shared_ptr<const BestFirstSearchParameters> itags_parameters,
            std::shared_ptr<const RobotTraitsMatrixReduction> robot_traits_matrix_reduction,
            std::shared_ptr<const SchedulerParameters> scheduler_parameters,
            std::vector<std::shared_ptr<const Task>> tasks,
            std::vector<std::shared_ptr<const Robot>> robots,
            std::vector<std::shared_ptr<const Species>> species,
            Eigen::MatrixXf team_traits_matrix,
            std::vector<std::shared_ptr<EnvironmentBase>> environments,
            std::vector<std::shared_ptr<MotionPlannerBase>> motion_planners,
            ConfigurationType task_configuration_type,
            OmplStateSpaceType ompl_state_space_type) :

            m_fcpop_parameters(fcpop_parameters),
            m_itags_parameters(m_itags_parameters),
            m_robot_traits_matrix_reduction(robot_traits_matrix_reduction),
            m_scheduler_parameters(scheduler_parameters),
            m_tasks(tasks),
            m_robots(robots),
            m_species(species),
            m_team_traits_matrix(team_traits_matrix),
            m_environments(environments),
            m_motion_planners(motion_planners),
            m_task_configuration_type(task_configuration_type),
            m_ompl_state_space_type(ompl_state_space_type) {}

    std::shared_ptr<GrstapsProblemInputs> GrstapsProblemInputs::spliceSpeciesAndMotionPlanners(
            const std::shared_ptr<const GrstapsProblemInputs> &for_mp_and_species) const {

        std::vector<std::shared_ptr<const Robot>> robots_new;
        std::map<std::shared_ptr<const Species>, std::shared_ptr<const Species>> species_map;
        for (int i = 0; i < m_species.size(); ++i) {
            for (int j = 0; j < for_mp_and_species->m_species.size(); ++j) {
                if (m_species[i]->name() == for_mp_and_species->m_species[j]->name()) {
                    species_map[m_species[i]] = for_mp_and_species->m_species[j];
                }
            }
        }
        for (int i = 0; i < robots().size(); ++i) {
            robots_new.push_back(std::make_shared<const Robot>(robots()[i]->name(),
                                                               robots()[i]->initialConfiguration(),
                                                               species_map[robots()[i]->species()]));
        }

        return std::make_shared<GrstapsProblemInputs>(
                m_fcpop_parameters,
                m_itags_parameters,
                m_robot_traits_matrix_reduction,
                m_scheduler_parameters,
                m_tasks,
                robots_new,
                //m_robots,
                for_mp_and_species->m_species,
                m_team_traits_matrix,
                m_environments,
                for_mp_and_species->m_motion_planners,
                m_task_configuration_type,
                m_ompl_state_space_type);
    }

    void GrstapsProblemInputs::checkConfiguration(const std::shared_ptr<const ConfigurationBase> &configuration) const {
        if (configuration->configurationType() != m_task_configuration_type) {
            throw createLogicError("Configuration type does not match the central one");
        }

        switch (m_task_configuration_type) {
            case ConfigurationType::e_ompl: {
                switch (m_ompl_state_space_type) {
                    case OmplStateSpaceType::e_se2: {
                        const auto &se2_configuration =
                                std::dynamic_pointer_cast<const Se2OmplConfiguration>(configuration);
                        if (!se2_configuration) {
                            throw createLogicError("Configuration state space type does not match the central one");
                        }
                        break;
                    }
                    case OmplStateSpaceType::e_se3: {
                        const auto &se3_configuration =
                                std::dynamic_pointer_cast<const Se3OmplConfiguration>(configuration);
                        if (!se3_configuration) {
                            throw createLogicError("Configuration state space type does not match the central one");
                        }
                        break;
                    }
                    default: {
                        throw createLogicError("Unknown ompl state space type");
                    }
                }
                break;
            }
            case ConfigurationType::e_graph: {
                throw createLogicError("Not Implemented");
                break;
            }
            default: {
                throw createLogicError("Unknown task configuration type");
            }
        }
    }

    void from_json(const nlohmann::json &j, GrstapsProblemInputs &problem_input) {
        // TODO(Andrew): Load PDDL files
        const std::string pddl_domain_filepath =
                j.at(constants::k_pddl).at(constants::k_domain_filepath).get<std::string>();
        const std::string pddl_problem_filepath =
                j.at(constants::k_pddl).at(constants::k_problem_filepath).get<std::string>();
        // Parse PDDL
        std::vector<std::shared_ptr<SasAction>> grounded_sas_actions;

        // Load Environments & Motion Planners
        problem_input.loadMotionPlanners(j.at(constants::k_motion_planners));

        // Load Task Associations
        problem_input.createTasks(grounded_sas_actions, j.at(constants::k_task_associations));

        // Load Species
        auto[name_to_species_mapping, num_traits] = problem_input.loadSpecies(j.at(constants::k_species));

        // Load Robots
        // Note: cannot use the normal from_json function because the vector of species is needed
        problem_input.loadRobots(name_to_species_mapping, num_traits, j.at(constants::k_robots));

        // Load Module Parameters
        {
            problem_input.m_fcpop_parameters =
                    j.at(constants::k_fcpop_parameters).get<std::shared_ptr<BestFirstSearchParameters>>();
            problem_input.m_itags_parameters =
                    j.at(constants::k_itags_parameters).get<std::shared_ptr<BestFirstSearchParameters>>();
            if (j.find(constants::k_robot_traits_matrix_reduction) != j.end()) {
                problem_input.m_robot_traits_matrix_reduction =
                        j.at(constants::k_robot_traits_matrix_reduction).get<std::shared_ptr<RobotTraitsMatrixReduction>>();
            } else {
                problem_input.m_robot_traits_matrix_reduction = std::make_shared<const RobotTraitsMatrixReduction>();
            }
            problem_input.m_scheduler_parameters =
                    SchedulerParameters::deserializeFromJson(j.at(constants::k_scheduler_parameters));
            // MP parameters are loaded up above
        }
    }

    void GrstapsProblemInputs::loadMotionPlanners(const nlohmann::json &j) {
        if (!j.is_array()) {
            throw createLogicError("'motion_planners' should be an array of objects");
        }

        for (const nlohmann::json &individual_mp: j) {
            m_environments.push_back(
                    EnvironmentBase::deserializeFromJson(individual_mp.at(constants::k_environment_parameters)));

            // First MP
            if (m_task_configuration_type == ConfigurationType::e_unknown) {
                m_task_configuration_type = m_environments.back()->configurationType();
            }
                // Any that do not agree with previous motion planners on the configuration space
            else if (m_task_configuration_type != m_environments.back()->configurationType()) {
                throw createLogicError("Cannot load environments of different configuration types");
            }

            auto mp_parameters = MotionPlannerParametersBase::loadJson(individual_mp.at(constants::k_mp_parameters));
            if (m_task_configuration_type != mp_parameters->configuration_type) {
                throw createLogicError("Cannot load mp parameters of different configuration type");
            }

            switch (m_task_configuration_type) {
                case ConfigurationType::e_ompl: {
                    loadOmplMotionPlanner(individual_mp, mp_parameters);
                    break;
                }
                case ConfigurationType::e_graph: {
                    throw createLogicError("Not implemented");
                }
                default: {
                    throw createLogicError("Unknown motion planner type");
                }
            }
        }
    }

    void GrstapsProblemInputs::loadOmplMotionPlanner(
            const nlohmann::json &j,
            const std::shared_ptr<const MotionPlannerParametersBase> &mp_parameters) {
        if (m_task_configuration_type != ConfigurationType::e_ompl) {
            throw createLogicError("Cannot load motion planners of different configuration types");
        }

        const OmplMotionPlannerType mp_type = j.at(constants::k_mp_type).get<OmplMotionPlannerType>();
        const std::shared_ptr<OmplEnvironment> &ompl_environment =
                std::dynamic_pointer_cast<OmplEnvironment>(m_environments.back());
        if (m_ompl_state_space_type == OmplStateSpaceType::e_unknown) {
            m_ompl_state_space_type = ompl_environment->stateSpaceType();
        } else if (m_ompl_state_space_type != ompl_environment->stateSpaceType()) {
            throw createLogicError("Cannot load OMPL environments with different state space types");
        }

        const std::shared_ptr<const OmplMotionPlannerParameters> &ompl_motion_planner_parameters =
                std::dynamic_pointer_cast<const OmplMotionPlannerParameters>(mp_parameters);
        m_motion_planners.push_back(
                std::make_shared<OmplMotionPlanner>(mp_type, ompl_motion_planner_parameters, ompl_environment));
    }

    void GrstapsProblemInputs::createTasks(const std::vector<std::shared_ptr<SasAction>> &grounded_sas_actions,
                                           const nlohmann::json &j) {
        m_tasks.reserve(grounded_sas_actions.size());
        for (const std::shared_ptr<SasAction> &action: grounded_sas_actions) {
            if (!j.contains(action->name())) {
                throw createLogicError(
                        fmt::format("No associated trait or geometric data for task '{0:s}'", action->name()));
            }

            const nlohmann::json &task_associations_j = j.at(action->name());

            const Eigen::VectorXf desired_traits =
                    task_associations_j.at(constants::k_desired_traits).get<Eigen::VectorXf>();
            const nlohmann::json &initial_configuration_j = task_associations_j.at(constants::k_initial_configuration);
            const nlohmann::json &terminal_configuration_j =
                    task_associations_j.at(constants::k_terminal_configuration);

            const std::shared_ptr<const ConfigurationBase> initial_configuration =
                    ConfigurationBase::deserializeFromJson(initial_configuration_j);
            checkConfiguration(initial_configuration);

            const std::shared_ptr<const ConfigurationBase> terminal_configuration =
                    ConfigurationBase::deserializeFromJson(terminal_configuration_j);
            checkConfiguration(terminal_configuration);

            m_tasks.push_back(
                    std::make_shared<const Task>(action, desired_traits, initial_configuration,
                                                 terminal_configuration));
        }
    }

    std::pair<std::map<std::string, std::shared_ptr<const Species>>, unsigned int> GrstapsProblemInputs::loadSpecies(
            const nlohmann::json &j) {
        if (m_motion_planners.empty()) {
            Logger::warn("Loading species without loading motion planners first");
        }

        std::map<std::string, std::shared_ptr<const Species>> rv;
        unsigned int num_traits;
        bool num_traits_set = false;

        m_species.reserve(j.size());
        for (const nlohmann::json &species_j: j) {
            m_species.push_back(Species::loadJson(species_j, m_motion_planners));
        }
        // Build map to use for loading the robots down below
        for (const std::shared_ptr<const Species> &s: m_species) {
            rv[s->name()] = s;
            if (!num_traits_set) {
                num_traits = s->traits().size();
                num_traits_set = true;
            }
        }

        return {rv, num_traits};
    }

    void GrstapsProblemInputs::loadRobots(
            const std::map<std::string, std::shared_ptr<const Species>> &name_to_species_mapping,
            const unsigned int num_traits,
            const nlohmann::json &j) {
        unsigned int num_robots = j.size();
        unsigned int robot_nr = 0;

        m_robots.reserve(num_robots);
        m_team_traits_matrix.resize(num_robots, num_traits);
        for (const nlohmann::json robot_j: j) {
            const std::string name = robot_j.at(constants::k_name).get<std::string>();
            std::shared_ptr<const ConfigurationBase> initial_configuration =
                    ConfigurationBase::deserializeFromJson(robot_j.at(constants::k_initial_configuration));
            const std::string species_name = robot_j.at(constants::k_species).get<std::string>();

            m_robots.push_back(
                    std::make_shared<const Robot>(name, initial_configuration,
                                                  name_to_species_mapping.at(species_name)));
            m_team_traits_matrix.row(robot_nr++) = m_robots.back()->species()->traits();
        }
    }

    GrstapsProblemInputs::~GrstapsProblemInputs() {
        for (std::shared_ptr<MotionPlannerBase> &mp: m_motion_planners) {
            mp->clearCache();
        }
    }
}  // namespace grstapse