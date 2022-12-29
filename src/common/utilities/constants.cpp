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
#include "grstapse/common/utilities/constants.hpp"

namespace grstapse::constants
{
    // json keys
    const char* k_allocation                            = "allocation";
    const char* k_alpha                                 = "alpha";
    const char* k_bounding_radius                       = "bounding_radius";
    const char* k_bounds                                = "bounds";
    const char* k_coalition                             = "coalition";
    const char* k_compute_transition_duration_heuristic = "compute_transition_duration_heuristic";
    const char* k_configuration_type                    = "configuration_type";
    const char* k_connection_range                      = "connection_range";
    const char* k_convergence_epislon                   = "convergence_epislon";
    const char* k_cost                                  = "cost";
    const char* k_desired_traits                        = "desired_traits";
    const char* k_domain_filepath                       = "domain_filepath";
    const char* k_dubins                                = "dubins";
    const char* k_duration                              = "duration";
    const char* k_edges                                 = "edges";
    const char* k_environment_parameters                = "environment_parameters";
    const char* k_environment_type                      = "environment_type";
    const char* k_execution_motion_plan                 = "execution_motion_plan";
    const char* k_fcpop_parameters                      = "fcpop_parameters";
    const char* k_finish_timepoint                      = "finish_timepoint";
    const char* k_goal_type                             = "goal_type";
    const char* k_graph_type                            = "graph_type";
    const char* k_heuristic_time                        = "heuristic_time";
    const char* k_high                                  = "high";
    const char* k_id                                    = "id";
    const char* k_image                                 = "image";
    const char* k_individual_plan                       = "individual_plan";
    const char* k_initial_configuration                 = "initial_configuration";
    const char* k_itags_parameters                      = "itags_parameters";
    const char* k_last_edge                             = "last_edge";
    const char* k_low                                   = "low";
    const char* k_makespan                              = "makespan";
    const char* k_worst_makespan                        = "worst_makespan";
    const char* k_milp_scheduler_type                   = "milp_scheduler_type";
    const char* k_mip_gap                               = "mip_gap";
    const char* k_motion_planners                       = "motion_planners";
    const char* k_motion_planning_time                  = "motion_planning_time";
    const char* k_mp_index                              = "mp_index";
    const char* k_mp_parameters                         = "mp_parameters";
    const char* k_mp_type                               = "mp_type";
    const char* k_mutex_constraints                     = "mutex_constraints";
    const char* k_name                                  = "name";
    const char* k_nodes_deadend                         = "nodes_deadend";
    const char* k_nodes_evaluated                       = "nodes_evaluated";
    const char* k_nodes_expanded                        = "nodes_expanded";
    const char* k_nodes_generated                       = "nodes_generated";
    const char* k_nodes_pruned                          = "nodes_pruned";
    const char* k_nodes_reopened                        = "nodes_reopened";
    const char* k_num_motion_plan_failures              = "num_motion_plan_failures";
    const char* k_num_motion_plans                      = "num_motion_plans";
    const char* k_num_scenarios                         = "num_scenarios";
    const char* k_num_scheduling_failures               = "num_scheduling_failures";
    const char* k_num_scheduling_iterations             = "num_scheduling_iterations";
    const char* k_origin                                = "origin";
    const char* k_path_cost_time                        = "path_cost_time";
    const char* k_pddl                                  = "pddl";
    const char* k_pgm_filepath                          = "pgm_filepath";
    const char* k_plan_task_indices                     = "plan_task_indices";
    const char* k_precedence_constraints                = "precedence_constraints";
    const char* k_precedence_set_mutex_constraints      = "precedence_set_mutex_constraints";
    const char* k_problem_filepath                      = "problem_filepath";
    const char* k_qw                                    = "qw";
    const char* k_qx                                    = "qx";
    const char* k_qy                                    = "qy";
    const char* k_qz                                    = "qz";
    const char* k_resolution                            = "resolution";
    const char* k_robot_traits_matrix_reduction         = "robot_traits_matrix_reduction";
    const char* k_robots                                = "robots";
    const char* k_rotation                              = "rotation";
    const char* k_scheduler_parameters                  = "scheduler_parameters";
    const char* k_scheduler_type                        = "scheduler_type";
    const char* k_scheduling_time                       = "scheduling_time";
    const char* k_simplify_path                         = "simplify_path";
    const char* k_simplify_path_timeout                 = "simplify_path_timeout";
    const char* k_solution                              = "solution";
    const char* k_solutions_window                      = "solutions_window";
    const char* k_species                               = "species";
    const char* k_speed                                 = "speed";
    const char* k_start_timepoint                       = "start_timepoint";
    const char* k_state_space_type                      = "state_space_type";
    const char* k_state_type                            = "state_type";
    const char* k_states                                = "states";
    const char* k_statistics                            = "statistics";
    const char* k_task_allocation_time                  = "task_allocation_time";
    const char* k_task_associations                     = "task_associations";
    const char* k_task_planning_time                    = "task_planning_time";
    const char* k_tasks                                 = "tasks";
    const char* k_terminal_configuration                = "terminal_configuration";
    const char* k_threads                               = "threads";
    const char* k_threshold                             = "threshold";
    const char* k_time                                  = "time";
    const char* k_timeout                               = "timeout";
    const char* k_total_time                            = "total_time";
    const char* k_traits                                = "traits";
    const char* k_transitions                           = "transitions";
    const char* k_turning_radius                        = "turning_radius";
    const char* k_use_hierarchical_objective            = "use_hierarchical_objective";
    const char* k_vector_reduction_function_type        = "vector_reduction_function_type";
    const char* k_vertex                                = "vertex";
    const char* k_vertex_a                              = "vertex_a";
    const char* k_vertex_b                              = "vertex_b";
    const char* k_vertices                              = "vertices";
    const char* k_x                                     = "x";
    const char* k_y                                     = "y";
    const char* k_yaml_filepath                         = "yaml_filepath";
    const char* k_yaw                                   = "yaw";
    const char* k_z                                     = "z";
}  // namespace grstapse::constants