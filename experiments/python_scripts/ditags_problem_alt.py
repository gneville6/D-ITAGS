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


import copy
import json
import os
import random

import cv2 as cv
import yaml

# Globals Used for Generation
num_probs = 200
append_problem_info = False

# Agent Parameters
lose_agent = False
gain_agent = False
range_agent_change = [1, 3]

# Agent Speed
speed_up = False
speed_down = False
range_speed_change = [0, 1]
range_num_speed_change = [1, 3]
speed_limits = [0.25, 8]

# Task
lose_task = False
gain_task = False
range_task_change = [1, 3]

# Task Durations
duration_up = False
duration_down = False
range_duration_change = [1, 4]
duration_limits = [0.0, 8]

# Agent Traits
traits_up = False
traits_down = False
range_trait_change = [1, 3]
trait_limits = [
    [0, 10],
    [0, 10],
    [0, 10],
    [0, 10],
    [0, 10],
    [0, 0]]

# Task Reqs
reqs_up = False
reqs_down = False
range_reqs_change = [1, 3]
req_limits = [
    [0, 11],
    [0, 11],
    [0, 11],
    [0, 11],
    [0, 11],
    [0, 0]]

# Directory that you want the problems stored to
file = "../Problems/WAFR/Base/"
dir = "/../Problems/WAFR/alts/alt"
base_loc = "../Problems/base_survivor_domain.json"

# PGM files denoting locations where task/agent can be placed
agent_loc_pgm = "/../maps/iros_map_part"
task_loc_pgm = "/../maps/iros_map_full_buildings"

experiment_tracker = [
    ["reqs_up"],
    ["reqs_down"],
    ["traits_up"],
    ["traits_down"],
    ["durations_up"],
    ["durations_down"],
    ["speed_up"],
    ["speed_down"],
    ["lost_agent"],
    ["gain_agent"],
    ["lost_task"],
    ["gain_task"]
]


# Map Showing Valid locations for task

# Map showing Valid locations for robots to start in


# Used for prep problem generation
def get_dir_ready(file_path):
    if lose_agent:
        file_path = file_path + "_lost_agent"
    if gain_agent:
        file_path = file_path + "_gained_agent"
    if traits_up:
        file_path = file_path + "_traits_up"
    if traits_down:
        file_path = file_path + "_traits_down"
    if lose_task:
        file_path = file_path + "_lost_task"
    if gain_task:
        file_path = file_path + "_gain_task"
    if reqs_up:
        file_path = file_path + "_reqs_up"
    if reqs_down:
        file_path = file_path + "_reqs_down"
    if speed_up:
        file_path = file_path + "_speed_up"
    if speed_down:
        file_path = file_path + "_speed_down"
    if duration_up:
        file_path = file_path + "_duration_up"
    if duration_down:
        file_path = file_path + "_duration_down"
    file_path = file_path + "/"
    cwd = os.getcwd()
    path = cwd + file_path
    if not os.path.exists(path):
        os.mkdir(path)
    return path


def load_base():
    with open(base_loc, "r") as read_file:
        base = json.load(read_file)
    return base


def load_prob(base, file_name, i):
    path = get_file_name(base, file_name, i)
    with open(path, "r") as read_file:
        base = json.load(read_file)
    return base


# Used for writing new problem to file
def get_file_name(prob, path, index):
    if append_problem_info:
        robot_str = "_" + str(len(prob["robots"])) + "_robots_"
        task_str = "_" + str(len(prob["tasks"])) + "_robots_"
        return path + 'survivor_problem' + str(index) + robot_str + task_str + '.json'
    else:
        return path + 'survivor_problem' + str(index) + '.json'


def get_copy_json_from_base(base):
    new = copy.deepcopy(base)
    return new


def write_problem_to_file(prob, path, index):
    with open(get_file_name(prob, path, index), 'w') as outfile:
        json.dump(prob, outfile, ensure_ascii=False, indent=4)


# Wrapped loop for creating many problems
def write_many_problem_from_base(num_problems):
    # Get the files ready to be written
    path = get_dir_ready(dir)
    global new_count
    for i in range(num_problems):
        new_count = 0
        base = load_base()
        new = load_prob(base, file, i)
        prob = create_new_problem_from_base(new, base)
        write_problem_to_file(prob, path, i)


def create_new_problem_from_base(new, base):
    if lose_agent or gain_agent:
        new = change_agents_count(new, base)
    if traits_up or traits_down:
        new = change_agents_traits(new, base)
    if lose_task or gain_task:
        new = change_task_count(new, base)
    if reqs_up or reqs_down:
        new = change_tasks_reqs(new, base)
    if speed_up or speed_down:
        new = change_agents_speed(new)
    if duration_up or duration_down:
        new = change_task_durations(new)
    return new


def change_agent_speed(new, species, up_down):
    new_speed = random.randrange(range_num_speed_change[0], range_num_speed_change[1])
    if up_down:
        new["species"][species]["speed"] = new["species"][species]["speed"] + new_speed
    else:
        new["species"][species]["speed"] = new["species"][species]["speed"] - new_speed
    if new["species"][species]["speed"] < speed_limits[0]:
        new["species"][species]["speed"] = speed_limits[0]
    if new["species"][species]["speed"] > speed_limits[1]:
        new["species"][species]["speed"] = speed_limits[1]
    return new


def change_agents_speed(new):
    current_changed = 0
    number_to_change = random.randrange(range_num_speed_change[0], range_num_speed_change[1])
    while current_changed != number_to_change:
        current_changed += 1
        species = random.randrange(0, len(new["species"]))
        if speed_up and speed_down:
            remove_or_add = random.randrange(0, 1)
            new = change_agent_speed(new, species, remove_or_add)
        elif speed_up:
            new = change_agent_speed(new, species, 1)
        elif speed_down:
            new = change_agent_speed(new, species, 0)
    return new


def change_agents_count(new, base):
    current_changed = 0
    number_to_change = random.randrange(range_agent_change[0], range_agent_change[1])
    while current_changed != number_to_change:
        current_changed += 1
        if lose_agent and gain_agent:
            should_gain = random.randrange(0, 1)
            if should_gain:
                new = add_agent_from_base(new, base)
            else:
                new = remove_agent(new, base)
        if gain_agent:
            new = add_agent_from_base(new, base)
        elif lose_agent:
            new = remove_agent(new, base)
    return new


def add_agent_from_base(new, base):
    agent_type_to_add = random.randrange(0, len(base["robots"]) - 1)
    new["robots"].append(copy.deepcopy(base["robots"][agent_type_to_add]))
    global new_count
    new["robots"][-1]["name"] = new["robots"][-1]["name"] + "New" + str(new_count)
    new_count += 1
    new = set_starting_locations_agent(new, -1)
    return new


def remove_agent(new, base):
    agent = random.randrange(0, len(new["robots"]))
    new["robots"][agent] = copy.deepcopy(base["robots"][-1])
    new["robots"][agent]["name"] = base["robots"][-1]["name"]
    return new


def change_task_count(new, base):
    current_changed = 0
    number_to_change = random.randrange(range_task_change[0], range_task_change[1])
    while current_changed != number_to_change:
        current_changed += 1
        if lose_task and gain_task:
            should_gain = random.randrange(0, 1)
            if should_gain:
                new = add_task_from_base(new, base)
            else:
                new = remove_task(new, base)
        if gain_task:
            new = add_task_from_base(new, base)
        elif lose_task:
            new = remove_task(new, base)
    return new


def set_task_init_location(new, task):
    loc = sample_task_location()
    new["tasks"][task]["initial_configuration"]["x"] = loc[0]
    new["tasks"][task]["initial_configuration"]["y"] = loc[1]
    new["tasks"][task]["initial_configuration"]["yaw"] = loc[2]
    return new


def set_task_fin_location(new, task):
    if new["action_finish"][new["tasks"][task]["name"]] == "none":
        new["tasks"][task]["terminal_configuration"]["x"] = new["tasks"][task]["initial_configuration"]["x"]
        new["tasks"][task]["terminal_configuration"]["y"] = new["tasks"][task]["initial_configuration"]["y"]
        new["tasks"][task]["terminal_configuration"]["yaw"] = new["tasks"][task]["initial_configuration"]["yaw"]
    else:
        for j in range(len(new["robots"])):
            if new["robots"][j]["name"] == new["action_finish"][new["tasks"][task]["name"]]:
                new["tasks"][task]["terminal_configuration"]["x"] = new["robots"][j]["initial_configuration"]["x"]
                new["tasks"][task]["terminal_configuration"]["y"] = new["robots"][j]["initial_configuration"]["y"]
                new["tasks"][task]["terminal_configuration"]["yaw"] = new["robots"][j]["initial_configuration"]["yaw"]

    return new


def remove_task(new, base):
    task = random.randrange(0, len(new["tasks"]))
    new["tasks"][task] = copy.deepcopy(base["tasks"][-1])
    return new


def add_task_from_base(new, base):
    task_to_add = random.randrange(0, len(base["tasks"]) - 1)
    new["tasks"].append(copy.deepcopy(base["tasks"][task_to_add]))
    new = add_plan_task_indices(new)
    new = set_starting_locations_agent(new, -1)
    new = set_task_fin_location(new, -1)
    prec_index = new["plan_task_indices"][-1]
    if str(task_to_add) in new["task_with_prec"]:
        for i in range(len(new["task_with_prec"][str(task_to_add)])):
            new["tasks"].append(copy.deepcopy(base["tasks"][new["task_with_prec"][str(task_to_add)][i]]))
            new = add_plan_task_indices(new)
            new["precedence_constraints"].append([prec_index, new["plan_task_indices"][-1]])
            new = set_starting_locations_agent(new, -1)
            new = set_task_fin_location(new, -1)
    return new


def add_plan_task_indices(new):
    new["plan_task_indices"].append(len(new["plan_task_indices"]))
    return new


def sample_task_location():
    im = cv.imread(task_loc_pgm + ".pgm", -1)
    row = random.randrange(0, im.shape[0])
    col = random.randrange(0, im.shape[1])
    while im[row][col] != 255:
        row = random.randrange(0, im.shape[0])
        col = random.randrange(0, im.shape[1])
    with open(task_loc_pgm + ".yaml") as file_pmg:
        yaml_config = yaml.load(file_pmg, Loader=yaml.FullLoader)
    x = (row - im.shape[0] / 2) * yaml_config["resolution"]
    y = (col - im.shape[1] / 2) * yaml_config["resolution"]

    return [x, y, 0.0]


def set_starting_locations_agent(new, agent):
    species_name = new["robots"][agent]["species"]
    for i in range(len(new["robots"])):
        if new["robots"][i]["species"] == species_name:
            new["robots"][agent]["initial_configuration"] = new["robots"][i]["initial_configuration"]
    return new


def change_task_durations(new):
    current_changed = 0
    number_to_change = random.randrange(range_duration_change[0], range_duration_change[1])
    while current_changed != number_to_change:
        task = random.randrange(0, len(new["tasks"]))
        current_changed += 1
        if duration_up and duration_down:
            should_raise = random.randrange(0, 1)
            if should_raise:
                new = change_task_duration(new, task, should_raise)
        if duration_up:
            new = change_task_duration(new, task, 1)
        elif duration_down:
            new = change_task_duration(new, task, 0)
    return new


def change_task_duration(new, task, should_raise):
    new_speed = random.randrange(range_num_speed_change[0], range_num_speed_change[1])
    if should_raise:
        new["tasks"][task]["duration"] = new["tasks"][task]["duration"] + new_speed
    else:
        new["tasks"][task]["duration"] = new["tasks"][task]["duration"] - new_speed

    if new["tasks"][task]["duration"] < duration_limits[0]:
        new["tasks"][task]["duration"] = duration_limits[0]
    if new["tasks"][task]["duration"] > duration_limits[1]:
        new["tasks"][task]["duration"] = duration_limits[1]
    return new


def change_agents_traits(new, base):
    current_changed = 0
    number_to_change = random.randrange(range_trait_change[0], range_trait_change[1])
    while current_changed != number_to_change:
        current_changed += 1

        if traits_up and traits_down:
            should_gain = random.randrange(0, 1)
            new = change_traits(new, base, should_gain)
        if traits_up:
            new = change_traits(new, base, 1)
        elif traits_down:
            new = change_traits(new, base, 0)
    return new


def change_tasks_reqs(new, base):
    current_changed = 0
    number_to_change = random.randrange(range_reqs_change[0], range_reqs_change[1])
    while current_changed != number_to_change:
        current_changed += 1

        if reqs_up and reqs_down:
            should_gain = random.randrange(0, 1)
            new = change_reqs(new, base, should_gain)
        if reqs_up:
            new = change_reqs(new, base, 1)
        elif reqs_down:
            new = change_reqs(new, base, 0)
    return new


def change_traits(new, base, should_raise):
    new_trait = random.randrange(range_trait_change[0], range_trait_change[1])
    species = random.randrange(0, len(base["species"]) - 1)

    trait = random.randrange(0, len(new["species"][0]["traits"]))

    if should_raise:
        new["species"][species]["traits"][trait] = new["species"][species]["traits"][trait] + new_trait
    else:
        new["species"][species]["traits"][trait] = new["species"][species]["traits"][trait] - new_trait

    if new["species"][species]["traits"][trait] < trait_limits[trait][0]:
        new["species"][species]["traits"][trait] = trait_limits[trait][0]
    if new["species"][species]["traits"][trait] > trait_limits[trait][1]:
        new["species"][species]["traits"][trait] = trait_limits[trait][1]
    return new


def change_reqs(new, base, should_raise):
    new_speed = random.randrange(range_reqs_change[0], range_reqs_change[1])
    task = random.randrange(0, len(base["tasks"]))
    trait = random.randrange(0, len(new["tasks"][0]["desired_traits"]))

    if should_raise:
        new["tasks"][task]["desired_traits"][trait] = new["tasks"][task]["desired_traits"][trait] + new_speed
    else:
        new["tasks"][task]["desired_traits"][trait] = new["tasks"][task]["desired_traits"][trait] - new_speed

    if new["tasks"][task]["desired_traits"][trait] < req_limits[trait][0]:
        new["tasks"][task]["desired_traits"][trait] = req_limits[trait][0]
    if new["tasks"][task]["desired_traits"][trait] > req_limits[trait][1]:
        new["tasks"][task]["desired_traits"][trait] = req_limits[trait][1]
    return new


def experiment_creator():
    # Agent Parameters
    global lose_agent
    global gain_agent

    # Agent Speed
    global speed_up
    global speed_down
    # Task
    global lose_task
    global gain_task

    # Task Durations
    global duration_up
    global duration_down

    # Agent Traits
    global traits_up
    global traits_down

    # Task Reqs
    global reqs_up
    global reqs_down

    for i in range(len(experiment_tracker)):
        experiment_decription = experiment_tracker[i]
        for j in experiment_decription:
            if j == "reqs_up":
                reqs_up = True
            if j == "reqs_down":
                reqs_down = True
            if j == "traits_up":
                traits_up = True
            if j == "traits_down":
                traits_down = True
            if j == "durations_up":
                # Task Durations
                duration_up = True
            if j == "durations_down":
                duration_down = True
            if j == "speed_up":
                speed_up = True
            if j == "speed_down":
                speed_down = True
            if j == "lost_task":
                lose_task = True
            if j == "gain_task":
                gain_task = True
            if j == "lost_agent":
                lose_agent = True
            if j == "gain_agent":
                gain_agent = True

        write_many_problem_from_base(num_probs)

        # Agent Parameters
        lose_agent = False
        gain_agent = False

        # Agent Speed
        speed_up = False
        speed_down = False

        # Task
        lose_task = False
        gain_task = False

        # Task Durations
        duration_up = False
        duration_down = False

        # Agent Traits
        traits_up = False
        traits_down = False

        # Task Reqs
        reqs_up = False
        reqs_down = False


def main():
    # Generate the problems
    experiment_creator()


if __name__ == "__main__":
    main()
