#  Graphically Recursive Simultaneous Task Allocation, Planning,
#  Scheduling, and Execution
#
#  Copyright (C) 2020-2022
#
#  Author: Andrew Messing
#  Author: Glen Neville
#
#  This program is free software: you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation, either version 3 of the License, or
#  (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#
#  You should have received a copy of the GNU General Public License
#  along with this program.  If not, see <https://www.gnu.org/licenses/>.


import json
import os
from os.path import exists

import numpy as np
import pandas as pd
from matplotlib import pyplot as plt
# Editable Fields
from matplotlib.colors import LinearSegmentedColormap

number_of_problems = 50

'''
"motion_planning_time",  # 1
"nodes_deadend",  # 2
"nodes_evaluated",  # 3
"nodes_expanded",  # 4
"nodes_generated",  # 5
"nodes_pruned",  # 6
"nodes_reopened",  # 7
"num_motion_plan_failures",  # 8
"num_motion_plans",  # 9
"num_scheduling_failures",  # 10
"num_scheduling_iterations",  # 11
"scheduling_time",  # 12
"task_allocation_time",  # 13
"total_time"  # 14
"makespan" #15
'''
stats_to_graph = [13, 14]  # This will create a subplot which each
'''
#1 "reqs_up/",
#2 "reqs_down/",
#3 "traits_up/",
#4 "traits_down/",
#5 "durations_up/",
#6 "speed_up/",
#7 "lost_agent/",
#8 "gain_agent/",
#9 "lost_task/",
#10 "gain_task/"
'''
alt_types = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9]
# which types of problems should be shown
problem_types_to_show = {"base": True, "repair": True, "replan": True}
sub_height = 3
sub_width = 3

# Directory Data
problems_directory = "/Problems/WAFR"
base_subfolder = ""
alt_subfolder = "alts/"
output_subfolder = "./outputs2"
problem_base_name = "survivor_problem"


def lines():
    print(os.getcwd())
    filenames = next(os.walk(f'{output_subfolder}/'), (None, None, []))[2]  # [] if no file
    # Non editable
    # For holding the data from JSON
    stats = [
        "motion_planning_time",  # 1
        "nodes_deadend",  # 2
        "nodes_evaluated",  # 3
        "nodes_expanded",  # 4
        "nodes_generated",  # 5
        "nodes_pruned",  # 6
        "nodes_reopened",  # 7
        "num_motion_plan_failures",  # 8
        "num_motion_plans",  # 9
        "num_scheduling_failures",  # 10
        "num_scheduling_iterations",  # 11
        "scheduling_time",  # 12
        "task_allocation_time",  # 13
        "total_time",  # 14
        "makespan"  # 15
    ]
    possible_alts = [
        "reqs_up",
        "reqs_down",
        "traits_up",
        "traits_down",
        "duration_up",
        "duration_down",
        "lost_agent",
        "gained_agent",
        "lost_task",
        "gain_task"
    ]

    y_axis_names = [
        "Motion Planning Time (s)",
        "Number of Deadend Nodes",
        "Number of Nodes Evaluated",
        "Number of Nodes Expanded",
        "Number of Nodes Generated",
        "Number of Nodes Pruned",
        "Number of Nodes Reopened",
        "Number of Motion Planning Failures",
        "Number of Motion Plans Generated",
        "Number of Scheduling Failures",
        "Number of Scheduling Iterations",
        "Scheduling Time (s)",
        "Task Allocation Time (s)",
        "Total Time (s)",
        "Makespan (s)"
    ]

    average = []
    for stat in stats_to_graph:
        fig, ax = plt.subplots(sub_height, sub_width)
        plot_num = 0
        plot_height = 0
        for alt_type in alt_types:
            base_data = []
            repair_data = []
            replan_data = []
            count = 1
            # print(count)
            for problem_nr in range(number_of_problems):
                path_out_base = f'{output_subfolder}/{problem_base_name}{problem_nr}.json'
                path_out_repair = f'{output_subfolder}/repair_{possible_alts[alt_type]}{problem_base_name}{problem_nr}.json'
                path_out_replan = f'{output_subfolder}/rerun_{possible_alts[alt_type]}{problem_base_name}{problem_nr}.json'
                if exists(path_out_replan) and exists(path_out_base) and exists(path_out_replan):
                    count += 1
                    if stat != 14:
                        with open(path_out_base) as f:
                            data = json.load(f)
                        base_data.append(data["statistics"][stats[stat]])

                        with open(path_out_repair) as f:
                            data = json.load(f)
                        repair_data.append(data["statistics"][stats[stat]])

                        with open(path_out_replan) as f:
                            data = json.load(f)
                        replan_data.append(data["statistics"][stats[stat]])
                    else:
                        with open(path_out_base) as f:
                            data = json.load(f)
                        base_data.append(data[stats[stat]])

                        with open(path_out_repair) as f:
                            data = json.load(f)
                        repair_data.append(data[stats[stat]])

                        with open(path_out_replan) as f:
                            data = json.load(f)
                        replan_data.append(data[stats[stat]])
                else:
                    i = 0
                    # print(path_out_base)
                    # print(path_out_replan)
                    # print(path_out_repair)

            xs = range(len(base_data))

            if plot_num == sub_width:
                plot_num = 0
                plot_height += 1
            size_legend = 0
            if problem_types_to_show["base"]:
                size_legend += 1
                ax[plot_height, plot_num].plot(xs, base_data, 'b', label='Base')
            if problem_types_to_show["repair"]:
                size_legend += 1
                ax[plot_height, plot_num].plot(xs, repair_data, 'r', label='Repair')
            if problem_types_to_show["replan"]:
                size_legend += 1
                ax[plot_height, plot_num].plot(xs, replan_data, 'g', label='Replan')

            # grid
            ax[plot_height, plot_num].grid(alpha=0.5, linestyle=':')

            # X/Y Axis
            ax[plot_height, plot_num].set_xlabel('Problem Number')
            if (plot_num == 0):
                ax[plot_height, plot_num].set_ylabel(y_axis_names[stat])
            plot_num += 1
            # Legend

        ax[0, 0].legend(bbox_to_anchor=(1.7, 1.7),
                        loc='upper center',
                        ncol=size_legend)
        width = 1.0
        height = 0.3

        margins = {
            "left": 0.1 / width,
            "right": 1.0 - 0.015 / width,
            "bottom": 0.06 / height,
            "top": 1.0 - 0.04 / height
        }
        width *= 7
        height *= 9
        plt.subplots_adjust(**margins)
        fig.set_size_inches(width, height)
        fig.savefig(f'{stats[stat]}.png', dpi=400)


def main():
    possible_alts_names = [
        "Requirements Up",
        "Requirements Down",
        "Agent Traits Up",
        "Agent Traits Down",
        "Task Duration Up",
        "Task Duration Down",
        "Lost Agent",
        "Gained Agent",
        "Lost Task",
        "Gain Task"
    ]

    print(os.getcwd())
    filenames = next(os.walk(f'{output_subfolder}/'), (None, None, []))[2]  # [] if no file
    # Non editable
    # For holding the data from JSON
    stats = [
        "motion_planning_time",  # 1
        "nodes_deadend",  # 2
        "nodes_evaluated",  # 3
        "nodes_expanded",  # 4
        "nodes_generated",  # 5
        "nodes_pruned",  # 6
        "nodes_reopened",  # 7
        "num_motion_plan_failures",  # 8
        "num_motion_plans",  # 9
        "num_scheduling_failures",  # 10
        "num_scheduling_iterations",  # 11
        "scheduling_time",  # 12
        "task_allocation_time",  # 13
        "total_time",  # 14
        "makespan"  # 15
    ]
    possible_alts = [
        "reqs_up",
        "reqs_down",
        "traits_up",
        "traits_down",
        "duration_up",
        "duration_down",
        "lost_agent",
        "gained_agent",
        "lost_task",
        "gain_task"
    ]
    y_axis_names = [
        "Motion Planning Time (s)",
        "Number of Deadend Nodes",
        "Number of Nodes Evaluated",
        "Number of Nodes Expanded",
        "Number of Nodes Generated",
        "Number of Nodes Pruned",
        "Number of Nodes Reopened",
        "Number of Motion Planning Failures",
        "Number of Motion Plans Generated",
        "Number of Scheduling Failures",
        "Number of Scheduling Iterations",
        "Scheduling Time (s)",
        "Task Allocation Time (s)",
        "Total Computation Time",
        "Makespan"
    ]

    for stat in stats_to_graph:
        average = []
        labels = []
        differences = []
        fig, ax = plt.subplots(2, 5)
        plt.suptitle(f"Comparision of Repair/Reallocation on {y_axis_names[stat]}")

        l = 0
        k = 0
        for alt_type in alt_types:

            base_data = []
            repair_data = []
            replan_data = []
            count = 1

            # print(count)
            for problem_nr in range(number_of_problems):
                path_out_base = f'{output_subfolder}/{problem_base_name}{problem_nr}.json'
                path_out_repair = f'{output_subfolder}/repair_{possible_alts[alt_type]}{problem_base_name}{problem_nr}.json'
                path_out_replan = f'{output_subfolder}/rerun_{possible_alts[alt_type]}{problem_base_name}{problem_nr}.json'
                if exists(path_out_replan) and exists(path_out_base) and exists(path_out_replan):
                    count += 1
                    if stat != 14:
                        with open(path_out_base) as f:
                            data = json.load(f)
                        base_data.append(data["statistics"][stats[stat]])

                        with open(path_out_repair) as f:
                            data = json.load(f)
                        repair_data.append(data["statistics"][stats[stat]])

                        with open(path_out_replan) as f:
                            data = json.load(f)
                        replan_data.append(data["statistics"][stats[stat]])
                    else:
                        with open(path_out_base) as f:
                            data = json.load(f)
                        base_data.append(data[stats[stat]])

                        with open(path_out_repair) as f:
                            data = json.load(f)
                        repair_data.append(data[stats[stat]])

                        with open(path_out_replan) as f:
                            data = json.load(f)
                        replan_data.append(data[stats[stat]])
                else:
                    i = 0
                    # print(path_out_base)
                    # print(path_out_replan)
                    # print(path_out_repair)

            if (len(replan_data)) > 1:
                labels.append(possible_alts[alt_type])

                cat_make = np.greater_equal(np.subtract(replan_data, repair_data), 0) * 1
                print(cat_make)
                data = pd.DataFrame({"X Value": repair_data, "Y Value": replan_data, "Category": cat_make})
                groups = data.groupby("Category")
                colors = [(1, 0, 0, 0.8), (0, 0.8, 0, 1)]
                # for name, group in groups:
                #    ax[l, k].plot(group["X Value"], group["Y Value"], marker="o", linestyle="",
                #                  color="b")  # colors[name]
                cat_make = np.subtract(replan_data, repair_data).astype(int)
                cmap = LinearSegmentedColormap.from_list("", ["red", "yellow", "green"])  # plt.cm.get_cmap('RdYlGn')
                most_diverged = max(-1 * np.min(cat_make), np.max(cat_make))
                norm = plt.Normalize(vmin=-1 * most_diverged, vmax=most_diverged)
                cat_make = cmap(norm(cat_make))
                ax[l, k].scatter(np.array(repair_data), np.array(replan_data), c=cat_make)
                # cmap=LinearSegmentedColormap.from_list("", ["green", "yellow", "red"]))  # colors[name]

                # Add some text for labels, title and custom x-axis tick labels, etc.
                ax[l, k].set_ylabel('ITAGS (s)')
                ax[l, k].set_title(f'{possible_alts_names[alt_type]}')
                ax[l, k].set_xlabel('D-ITAGS (s)')
                if (stat == 13):
                    xy_line = np.arange(0, 200)
                    ax[l, k].set_xlim(0, 200)
                    ax[l, k].set_ylim(0, 200)
                elif (stat == 14):
                    xy_line = np.arange(0, 2000)
                    ax[l, k].set_xlim(0, 2000)
                    ax[l, k].set_ylim(0, 2000)
                else:
                    xy_line = np.arange(0, 200)
                    ax[l, k].set_xlim(0, 200)
                    ax[l, k].set_ylim(0, 200)
                ax[l, k].plot(xy_line, '#bbbbbb', ls="--", label='Equal Makespan')

                fig.tight_layout()

                print(str(i) + ":" + str(k))
                k += 1
                if (k == 5):
                    k = 0
                    l += 1

        plt.show()


if __name__ == '__main__':
    main()
