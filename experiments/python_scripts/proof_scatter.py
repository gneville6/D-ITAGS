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

import numpy as np
from matplotlib import pyplot as plt

# Editable Fields

number_of_problems = 8

# Directory Data
problems_directory = "/Problems/WAFR_Proof/"
base_subfolder = "Base"
alt_subfolder = "alts/"
output_subfolder = "./outputs_large"
problem_base_name = "survivor_problem"

alphas = ["0.000000", "0.100000", "0.200000", "0.300000", "0.400000", "0.500000", "0.600000", "0.700000", "0.800000",
          "0.900000", "1.000000"]
problem_nums = [3, 4, 5, 6, 8, 9, 12, 16, 18, 19, 21, 22, 23, 24, 25, 26, 28, 29, 33, 46, 55, 48, 49, 59, 60, 62, 64,
                65, 71, 75, 77, 78, 87, 89, 92, 94, 99]


def main():
    average = []
    labels = []
    differences = []
    x = 11
    fig, ax = plt.subplots(1, 1)
    plt.suptitle(f"Validation of Bounds on Makespan")
    ax.set_xlabel('Alpha Value')
    ax.set_ylabel('Normalized time-optimality gap (Makespan)')

    all_makespan_data = []
    all_allocation_data = []
    for problem_nr in problem_nums:
        makespan_data = []
        allocation_data = []
        for alpha in alphas:
            path_out_base = f'{output_subfolder}/alpha_{alpha}{problem_base_name}{problem_nr}.json'

            with open(path_out_base) as f:
                data = json.load(f)
                makespan_data.append(data["makespan"])
                allocation_data.append(np.sum(data["allocation"]))
        all_makespan_data.append(makespan_data)
        all_allocation_data.append((allocation_data))

    violins = []
    for i in range(len(alphas)):
        alpha_range = []
        for prob in all_makespan_data:
            alpha_range.append((prob[i] - prob[0]) / (prob[-1] - prob[0]))
        violins.append(alpha_range)
    float_alpha = [0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0]
    ax.violinplot(violins[0:x], float_alpha[0:x], widths=0.1,
                  showmeans=False, showmedians=False, showextrema=False)

    for i in range(len(all_makespan_data)):
        data = [(datum - all_makespan_data[i][0]) / (all_makespan_data[i][-1] - all_makespan_data[i][0]) for datum in
                all_makespan_data[i]]
        if i == len(all_makespan_data) - 1:
            plt.scatter(alphas[0:x], data[0:x], color='cadetblue', label="Actual optimality gap")
        else:
            plt.scatter(alphas[0:x], data[0:x], color='cadetblue')

    data = []
    for j in alphas:
        if (float(j) == 0):
            data.append(0)
        elif float(j) < 0.5:
            data.append(float(j) / (1 - float(j)))
        else:
            data.append(1)
    plt.plot(alphas[0:x], data[0:x], label=f"Theoretical bound")
    plt.legend()

    plt.show()


def alloc():
    average = []
    labels = []
    differences = []
    x = 11
    fig, ax = plt.subplots(1, 1)
    plt.suptitle(f"Validation of Bounds on Resource Allocation")
    ax.set_xlabel('Alpha Value')
    ax.set_ylabel('Normalized resource-optimality gap (Allocation)')

    all_makespan_data = []
    all_allocation_data = []
    for problem_nr in problem_nums:
        makespan_data = []
        allocation_data = []
        for alpha in alphas:
            path_out_base = f'{output_subfolder}/alpha_{alpha}{problem_base_name}{problem_nr}.json'

            with open(path_out_base) as f:
                data = json.load(f)
                makespan_data.append(data["makespan"])
                allocation_data.append(np.sum(data["allocation"]))
        all_makespan_data.append(makespan_data)
        all_allocation_data.append((allocation_data))

    violins = []
    for i in range(len(alphas)):
        alpha_range = []
        for prob in all_allocation_data:
            alpha_range.append((prob[i] - prob[-1]))
        violins.append(alpha_range)
    float_alpha = [0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0]
    ax.violinplot(violins[0:x], float_alpha[0:x], widths=0.1,
                  showmeans=False, showmedians=False, showextrema=False)

    for i in range(len(all_allocation_data)):
        # (datum - all_allocation_data[i][-1]) / (all_allocation_data[i][0] - all_allocation_data[i][-1])
        data = [datum - all_allocation_data[i][-1] for datum in all_allocation_data[i]]
        if i == len(all_allocation_data) - 1:
            plt.scatter(alphas[0:x], data[0:x], color="cadetblue", label="Actual optimality gap")
        else:
            plt.scatter(alphas[0:x], data[0:x], color="cadetblue")

    data = []
    for j in alphas:
        if (float(j) == 0):
            data.append(0)
        elif float(j) < 0.5:
            data.append(float(j) / (1 - float(j)))
        else:
            data.append(1)
    # plt.plot(alphas[0:x], data[0:x], label=f"Theoretical bound")
    plt.legend()

    plt.show()


def differnce():
    average = []
    labels = []
    differences = []

    fig, ax = plt.subplots(1, 1)
    plt.suptitle(f"Difference Between Theoretical Bound and Solution Makespan")

    all_makespan_data = []
    all_allocation_data = []
    for problem_nr in problem_nums:
        makespan_data = []
        allocation_data = []
        for alpha in alphas:
            path_out_base = f'{output_subfolder}/alpha_{alpha}{problem_base_name}{problem_nr}.json'

            with open(path_out_base) as f:
                data = json.load(f)
                makespan_data.append(data["makespan"])
                allocation_data.append(np.sum(data["allocation"]))
        all_makespan_data.append(makespan_data)
        all_allocation_data.append((allocation_data))

    difference = []
    count = 0
    for problem in all_makespan_data:
        alpha_variation = []
        for j in range(len(alphas)):
            alpha = float(alphas[j])
            if alpha <= 0.5:
                sub_proof_val = ((alpha / (1 - alpha)) * problem[-1]) - problem[j] + problem[0]
            else:
                sub_proof_val = problem[-1] - problem[j]
            if sub_proof_val < 0:
                print(f"{alpha}, {problem_nums[count]}")
            alpha_variation.append(sub_proof_val)
        difference.append(alpha_variation)
        count += 1
    label = 0
    for i in range(len(difference)):
        plt.scatter(alphas, difference[i], label=f"problem {label}")
        label += 1
        # plt.plot(alphas, data, label=f"Suboptimality Gaurentee")
    plt.legend()
    plt.show()


if __name__ == '__main__':
    alloc()
    main()
    differnce()
