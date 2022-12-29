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
from os.path import exists

import numpy as np
import pandas as pd
from matplotlib import pyplot as plt

# Editable Fields
number_of_problems = 25

# Directory Data
output_subfolder = "./comp_output"


def main():
    average = []
    labels = []
    differences = []

    itags_time_data = []
    ditags_time_data = []
    itags_make_data = []
    ditags_make_data = []
    count = 1

    # print(count)
    for problem_nr in range(number_of_problems):
        path_out_itags = f'{output_subfolder}/itags_output_old_{problem_nr}.json'
        path_out_ditags = f'{output_subfolder}/out_survivor_problem{problem_nr}.json'

        if exists(path_out_itags) and exists(path_out_ditags):
            count += 1
            with open(path_out_itags) as f:
                data = json.load(f)
            itags_time_data.append(data["timer"])
            itags_make_data.append(data["makespan"])

            with open(path_out_ditags) as f:
                data = json.load(f)
            ditags_time_data.append(data["statistics"]["total_time"])
            ditags_make_data.append(data["makespan"])
        else:
            print("Failed on problem" + str(problem_nr))

    zip_time = zip(itags_time_data, ditags_time_data)
    difference_time = []
    for list1_i, list2_i in zip_time:
        difference_time.append(100 * (list2_i - list1_i) / list2_i)

    zip_make = zip(itags_make_data, ditags_make_data)
    difference_make = []
    for list1_i, list2_i in zip_make:
        difference_make.append(100 * (list2_i - list1_i) / list2_i)

    differences = [difference_make]  # difference_time,

    x = np.arange(1, 2)  # the label locations
    width = 0.35  # the width of the bars

    fig, ax = plt.subplots()
    ax.boxplot(differences, showfliers=False)
    print(differences)
    # Add some text for labels, title and custom x-axis tick labels, etc.
    ax.set_ylabel(f'Relative Percent Change')
    ax.set_title('Comparison of ITAGS to D-ITAGS')

    plt.xticks([1], ["Makespan"])  # "Computation Time",

    plt.axhline(y=0, linewidth=1, color='k')

    fig.tight_layout()

    plt.show()


def lineChartComp():
    average = []
    labels = []
    differences = []

    itags_time_data = []
    ditags_time_data = []
    itags_make_data = []
    ditags_make_data = []
    count = 1

    # print(count)
    for problem_nr in range(number_of_problems):
        path_out_itags = f'{output_subfolder}/itags_output_old_{problem_nr}.json'
        path_out_ditags = f'{output_subfolder}/out_survivor_problem{problem_nr}.json'

        if exists(path_out_itags) and exists(path_out_ditags):
            count += 1
            with open(path_out_itags) as f:
                data = json.load(f)
            itags_time_data.append(data["timer"])
            itags_make_data.append(data["makespan"])

            with open(path_out_ditags) as f:
                data = json.load(f)
            ditags_time_data.append(data["statistics"]["total_time"])
            ditags_make_data.append(data["makespan"])
        else:
            print("Failed on problem" + str(problem_nr))

    z = np.arange(1, len(ditags_time_data))
    fig, ax = plt.subplots()

    cat_make = np.greater_equal(np.subtract(itags_time_data, ditags_time_data), 0) * 1
    print(cat_make)
    data = pd.DataFrame({"X Value": ditags_time_data, "Y Value": itags_time_data, "Category": cat_make})
    groups = data.groupby("Category")
    colors = [(1, 0, 0, 0.8), (0, 0.8, 0, 1)]
    for name, group in groups:
        plt.plot(group["X Value"], group["Y Value"], marker="o", linestyle="", color=colors[name])
    # ax.scatter(ditags_make_data, itags_make_data, c='blue')

    # create the line
    xy_line = np.arange(0, 250)
    ax.plot(xy_line, '#bbbbbb', ls="--", label='Equal Makespan')
    # ax.annotate('Better', xy=(120, 120), xytext=(120, 140),
    #            arrowprops=dict(arrowstyle='<-'), ha='center', va='center')
    # ax.annotate('Worse', xy=(120, 120), xytext=(120, 100),
    #            arrowprops=dict(arrowstyle='<-'), ha='center', va='center')
    # add labels, legend and make it nicer
    ax.set_xlabel('D-ITAGS')
    ax.set_ylabel('ITAGS')
    ax.set_title('Computation Time (s)')
    ax.set_xlim(0, 150)
    ax.set_ylim(0, 150)
    ax.set_aspect('equal', adjustable='box')
    ax.legend()
    plt.tight_layout()

    plt.show()


def lineChartMake():
    average = []
    labels = []
    differences = []

    itags_time_data = []
    ditags_time_data = []
    itags_make_data = []
    ditags_make_data = []
    count = 1

    # print(count)
    for problem_nr in range(number_of_problems):
        path_out_itags = f'{output_subfolder}/itags_output_old_{problem_nr}.json'
        path_out_ditags = f'{output_subfolder}/out_survivor_problem{problem_nr}.json'

        if exists(path_out_itags) and exists(path_out_ditags):
            count += 1
            with open(path_out_itags) as f:
                data = json.load(f)
            itags_time_data.append(data["timer"])
            itags_make_data.append(data["makespan"])

            with open(path_out_ditags) as f:
                data = json.load(f)
            ditags_time_data.append(data["statistics"]["total_time"])
            ditags_make_data.append(data["makespan"])
        else:
            print("Failed on problem" + str(problem_nr))

    z = np.arange(1, len(ditags_time_data))
    fig, ax = plt.subplots()

    cat_make = np.greater_equal(np.subtract(itags_make_data, ditags_make_data), 0) * 1
    print(cat_make)
    data = pd.DataFrame({"X Value": ditags_make_data, "Y Value": itags_make_data, "Category": cat_make})
    groups = data.groupby("Category")
    colors = [(1, 0, 0, 0.8), (0, 0.8, 0, 1)]
    for name, group in groups:
        plt.plot(group["X Value"], group["Y Value"], marker="o", linestyle="", color=colors[name])
    # ax.scatter(ditags_make_data, itags_make_data, c='blue')

    # create the line
    xy_line = np.arange(0, 1500)
    ax.plot(xy_line, '#bbbbbb', ls="--", label='Equal Makespan')
    # ax.annotate('Better', xy=(120, 120), xytext=(120, 140),
    #            arrowprops=dict(arrowstyle='<-'), ha='center', va='center')
    # ax.annotate('Worse', xy=(120, 120), xytext=(120, 100),
    #            arrowprops=dict(arrowstyle='<-'), ha='center', va='center')
    # add labels, legend and make it nicer
    ax.set_xlabel('D-ITAGS')
    ax.set_ylabel('ITAGS')
    ax.set_title('Makespan (s)')
    ax.set_xlim(0, 1500)
    ax.set_ylim(0, 1500)
    ax.set_aspect('equal', adjustable='box')
    ax.legend()
    plt.tight_layout()

    plt.show()


if __name__ == '__main__':
    lineChartComp()
    lineChartMake()
    main()
