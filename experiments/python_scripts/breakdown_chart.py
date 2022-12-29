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

from matplotlib import pyplot as plt

# Editable Fields
number_of_problems = 200

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
'''
stats_to_graph = [14]  # This will create a subplot which each
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
alt_types = [1]
# which types of problems should be shown
problem_types_to_show = {"base": True, "repair": True, "replan": True}
sub_height = 1
sub_width = 1

# Directory Data
problems_directory = ""
base_subfolder = "Base/"
alt_subfolder = "alts/"
output_subfolder = "output/"
problem_base_name = "survivor_problem"


def main():
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
        "total_time"  # 14
    ]
    possible_alts = [
        "reqs_up/",
        "reqs_down/",
        "traits_up/",
        "traits_down/",
        "durations_up/",
        "speed_up/",
        "lost_agent/",
        "gain_agent/",
        "lost_task/",
        "gain_task/"
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
        "Total Time (s)"
    ]

    for alt_type in alt_types:
        base_data = []
        repair_data = []
        replan_data = []
        for problem_nr in range(number_of_problems):
            path_out_base = f'{problems_directory}/{output_subfolder}/{base_subfolder}/{problem_base_name}{problem_nr}.json'
            path_out_repair = f'{problems_directory}/{output_subfolder}/{alt_type}/repair_{problem_base_name}{problem_nr}.json'
            path_out_replan = f'{problems_directory}/{output_subfolder}/{alt_type}/rerun_{problem_base_name}{problem_nr}.json'

            with open(path_out_base) as f:
                data = json.load(f)
            base_data.append(data["statistics"][stat])

            with open(path_out_repair) as f:
                data = json.load(f)
            repair_data.append(data["statistics"][stat])

            with open(path_out_replan) as f:
                data = json.load(f)
            replan_data.append(data["statistics"][stat])

        fig, ax = plt.subplots()

        xs = range(1, 51)

        plt.subplot(sub_height, sub_width, plot_num)
        plot_num += 1
        size_legend = 0
        if problem_types_to_show["base"]:
            size_legend += 1
            ax.plot(xs, base_data, 'b', label='Base')
        if problem_types_to_show["repair"]:
            size_legend += 1
            ax.plot(xs, repair_data, 'r', label='Repair')
        if problem_types_to_show["replan"]:
            size_legend += 1
            ax.plot(xs, replan_data, 'g', label='Replan')

        # grid
        ax.grid(alpha=0.5, linestyle=':')

        # X/Y Axis
        ax.set_xlabel('Problem Number')
        ax.set_ylabel(y_axis_names[stat])

        # Legend
        ax.legend(bbox_to_anchor=(0.5, 1.2),
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
    fig.savefig(f'comp_time.png', dpi=400)


if __name__ == '__main__':
    main()
