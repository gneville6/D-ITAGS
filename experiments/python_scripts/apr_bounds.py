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
import math

import numpy as np
from matplotlib import pyplot as plt

# Editable Fields

number_of_problems = 8

# Directory Data
problems_directory = "/Problems/WAFR_Proof/"
base_subfolder = "Base"
alt_subfolder = "alts/"
output_subfolder = "./apr_results"
problem_base_name = "survivor_problem"

alphas = ["0.510000", "0.530000", "0.550000", "0.570000", "0.590000", "0.610000", "0.630000", "0.650000", "0.670000",
          "0.690000", "0.710000", "0.730000", "0.750000", "0.770000", "0.790000", "0.810000", "0.830000", "0.850000",
          "0.870000", "0.890000", "0.910000", "0.930000", "0.950000",
          "0.970000", "1.000000"]
problem_nums = [2, 8, 11, 15, 16, 18, 20, 23, 28, 30, 32, 38, 40, 42, 45, 46, 47, 50, 51, 52, 53]
# [0, 2, 6, 8, 9, 11, 12, 13, 15, 16, 17, 18, 20, 23, 24, 26, 27, 28, 30, 32, 33, 34, 36, 38, 40, 41, 42, 44, 45, 46, 47, 49, 50, 51, 52, 53]
delta_modifier = 1.5
total_problem = 0
current_problem = problem_nums[0]


def get_ancestor_node(opt_solution, current_solution):
    opt_alloc = np.array(opt_solution["allocation"])
    current_alloc = np.array(current_solution["allocation"])
    ancestor_alloc = np.logical_and(opt_alloc, current_alloc).astype(int)
    return ancestor_alloc


def get_y_star(problem):
    y_star = []
    for task in problem["tasks"]:
        y_star.append(task["desired_traits"])
    return y_star


def get_traits(problem):
    traits = []
    for robot in problem['robots']:
        for species in problem["species"]:
            if species["name"] == robot["species"]:
                traits.append(species["traits"])
    return traits


def get_delta(y_star):
    return delta_modifier / np.sum(y_star)


def get_apr(y_star, node, traits):
    alloc_trait_mat = np.matmul(np.array(node), np.array(traits))
    sub = np.subtract(y_star, alloc_trait_mat)
    zeroes = np.zeros(sub.shape)
    remove_neg_sum = np.sum(np.maximum(sub, zeroes))
    apr = remove_neg_sum / np.sum(y_star)
    return apr


def get_bounds(problem, opt_solution, current_solution, alpha):
    ancestor_node = get_ancestor_node(opt_solution, current_solution)
    y_star = get_y_star(problem)
    traits = get_traits(problem)
    delta = get_delta(y_star)
    apr = get_apr(y_star, ancestor_node, traits)
    if alpha > 0.5:
        bounds = math.ceil((1 - alpha) / (alpha * delta) + apr / delta)
        alt_bounds = np.sum(current_solution["allocation"]) - len(problem["tasks"])
        if alt_bounds < bounds:
            alloc = current_solution["allocation"]
            length = len(problem["tasks"])
            print(f"bounds = {alt_bounds}   {alloc} ,  {length}")
            bounds = alt_bounds
            global total_problem
            total_problem += 1
        else:
            bounds = bounds
            print(current_problem)
            print(alpha)
            print(bounds)
    else:
        bounds = np.sum(current_solution["allocation"]) - len(problem["tasks"])
    return bounds


def main():
    theoretic_bounds = []
    for problem_nr in problem_nums:
        global current_problem
        current_problem = problem_nr
        problem_bounds = []
        path_out_base = f'..{problems_directory}Very_large/{problem_base_name}{problem_nr}.json'
        with open(path_out_base) as f:
            problem = json.load(f)
        path_out_base = f'{output_subfolder}/alpha_{"1.000000"}{problem_base_name}{problem_nr}.json'
        with open(path_out_base) as f:
            opt_solution = json.load(f)
        for alpha in alphas:
            if alpha == "1.000000":
                problem_bounds.append(0)
            else:
                path_out_base = f'{output_subfolder}/alpha_{alpha}{problem_base_name}{problem_nr}.json'
                with open(path_out_base) as f:
                    current_solution = json.load(f)
                    bound = get_bounds(problem, opt_solution, current_solution, float(alpha))
                problem_bounds.append(bound)
        theoretic_bounds.append(problem_bounds)
    return theoretic_bounds


def alloc(bounds):
    average = []
    labels = []
    differences = []
    x = 25
    fig, ax = plt.subplots(1, 1)
    plt.suptitle(f"Validation of Bounds on Resource Allocation")

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
    float_alpha = np.array(
        ["0.510000", "0.530000", "0.550000", "0.570000", "0.590000", "0.610000", "0.630000", "0.650000", "0.670000",
         "0.690000", "0.710000", "0.730000", "0.750000", "0.770000", "0.790000", "0.810000", "0.830000", "0.850000",
         "0.870000", "0.890000", "0.910000", "0.930000", "0.950000",
         "0.970000", "1.000000"]).astype(float)
    ax.violinplot(violins[0:x], float_alpha[0:x], widths=0.01,
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
            plt.plot(alphas[0:x], bounds[i][0:x], label=f"Theoretical bound")
        ax.legend()
        ax.set_xlabel('Alpha Value')
        ax.set_ylabel('Normalized resource-optimality gap (Allocation)')
        print(problem_nums[i])
        plt.show()


if __name__ == '__main__':
    bounds = main()
    alloc(bounds)
