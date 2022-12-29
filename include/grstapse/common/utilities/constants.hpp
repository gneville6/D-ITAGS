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
#pragma once

namespace grstapse::constants
{
    // Json Keys
    extern const char* k_allocation;
    extern const char* k_alpha;
    extern const char* k_bounding_radius;
    extern const char* k_bounds;
    extern const char* k_coalition;
    extern const char* k_compute_transition_duration_heuristic;
    extern const char* k_configuration_type;
    extern const char* k_connection_range;
    extern const char* k_convergence_epislon;
    extern const char* k_cost;
    extern const char* k_desired_traits;
    extern const char* k_domain_filepath;
    extern const char* k_dubins;
    extern const char* k_duration;
    extern const char* k_edges;
    extern const char* k_environment_parameters;
    extern const char* k_environment_type;
    extern const char* k_execution_motion_plan;
    extern const char* k_fcpop_parameters;
    extern const char* k_finish_timepoint;
    extern const char* k_goal_type;
    extern const char* k_graph_type;
    extern const char* k_heuristic_time;
    extern const char* k_high;
    extern const char* k_id;
    extern const char* k_image;
    extern const char* k_individual_plan;
    extern const char* k_initial_configuration;
    extern const char* k_itags_parameters;
    extern const char* k_last_edge;
    extern const char* k_low;
    extern const char* k_makespan;
    extern const char* k_worst_makespan;
    extern const char* k_milp_scheduler_type;
    extern const char* k_mip_gap;
    extern const char* k_motion_planners;
    extern const char* k_motion_planning_time;
    extern const char* k_mp_index;
    extern const char* k_mp_parameters;
    extern const char* k_mp_type;
    extern const char* k_mutex_constraints;
    extern const char* k_name;
    extern const char* k_nodes_deadend;
    extern const char* k_nodes_evaluated;
    extern const char* k_nodes_expanded;
    extern const char* k_nodes_generated;
    extern const char* k_nodes_pruned;
    extern const char* k_nodes_reopened;
    extern const char* k_num_motion_plan_failures;
    extern const char* k_num_motion_plans;
    extern const char* k_num_scenarios;
    extern const char* k_num_scheduling_failures;
    extern const char* k_num_scheduling_iterations;
    extern const char* k_origin;
    extern const char* k_path_cost_time;
    extern const char* k_pddl;
    extern const char* k_pgm_filepath;
    extern const char* k_plan_task_indices;
    extern const char* k_precedence_constraints;
    extern const char* k_precedence_set_mutex_constraints;
    extern const char* k_problem_filepath;
    extern const char* k_qw;
    extern const char* k_qx;
    extern const char* k_qy;
    extern const char* k_qz;
    extern const char* k_resolution;
    extern const char* k_robot_traits_matrix_reduction;
    extern const char* k_robots;
    extern const char* k_rotation;
    extern const char* k_scheduler_parameters;
    extern const char* k_scheduler_type;
    extern const char* k_scheduling_time;
    extern const char* k_simplify_path;
    extern const char* k_simplify_path_timeout;
    extern const char* k_solution;
    extern const char* k_solutions_window;
    extern const char* k_species;
    extern const char* k_speed;
    extern const char* k_start_timepoint;
    extern const char* k_state_space_type;
    extern const char* k_state_type;
    extern const char* k_states;
    extern const char* k_statistics;
    extern const char* k_task_allocation_time;
    extern const char* k_task_associations;
    extern const char* k_task_planning_time;
    extern const char* k_tasks;
    extern const char* k_terminal_configuration;
    extern const char* k_threads;
    extern const char* k_threshold;
    extern const char* k_time;
    extern const char* k_timeout;
    extern const char* k_total_time;
    extern const char* k_traits;
    extern const char* k_transitions;
    extern const char* k_turning_radius;
    extern const char* k_use_hierarchical_objective;
    extern const char* k_vector_reduction_function_type;
    extern const char* k_vertex;
    extern const char* k_vertex_a;
    extern const char* k_vertex_b;
    extern const char* k_vertices;
    extern const char* k_x;
    extern const char* k_y;
    extern const char* k_yaml_filepath;
    extern const char* k_yaw;
    extern const char* k_z;
}  // namespace grstapse::constants