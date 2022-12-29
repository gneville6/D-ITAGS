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


import copy
import json
import os
import random

import cv2 as cv
import yaml

# Globals Used for Generation
num_probs = 25
append_problem_info = False

# Agent Parameters
num_agent_min = 8
num_agent_max = 12
num_type_robot_min = [1, 1, 1, 0, 0, 0]
num_type_robot_max = [3, 3, 3, 1, 1, 0]

# Task Parameters
num_task_min = 1
num_task_max = 15

# Directory that you want the problems stored to
file = "/../Problems/WAFR/Compare/"
base_loc = "../Problems/revised_base_survivor_domain.json"

base_loc_old = "../Problems/revised_base_survivor_domain_old.json"

# PGM files denoting locations where task/agent can be placed
agent_loc_pgm = "../maps/iros_map_part"
task_loc_pgm = "../maps/iros_map_full_buildings"


# Map Showing Valid locations for task

# Map showing Valid locations for robots to start in


# Used for prep problem generation
def get_dir_ready(file_path):
    cwd = os.getcwd()
    path = cwd + file_path
    if not os.path.exists(path):
        os.mkdir(path)
    return path


def load_base():
    with open(base_loc, "r") as read_file:
        base = json.load(read_file)
    return base


# Used for writing new problem to file
def get_file_name(prob, path, index, new_or_old):
    if (new_or_old):
        if (append_problem_info):
            robot_str = "_" + str(len(prob["robots"])) + "_robots_"
            task_str = "_" + str(len(prob["tasks"])) + "_robots_"
            return path + 'survivor_problem' + str(index) + robot_str + task_str + '.json'
        else:
            return path + 'survivor_problem' + str(index) + '.json'
    else:
        if (append_problem_info):
            robot_str = "_" + str(len(prob["robots"])) + "_robots_"
            task_str = "_" + str(len(prob["tasks"])) + "_robots_"
            return path + 'survivor_problem_old' + str(index) + robot_str + task_str + '.json'
        else:
            return path + 'survivor_problem_old' + str(index) + '.json'


def write_problem_to_file(prob, path, index, new_or_old):
    with open(get_file_name(prob, path, index, new_or_old), 'w') as outfile:
        json.dump(prob, outfile, ensure_ascii=False, indent=4)


# Wrapped loop for creating many problems
def write_many_problem_from_base(num_problems):
    # Loads the base which will be used to construct the new problems
    base = load_base()

    # Used to make sure global bounds are met
    global current_agent_type_count

    # Get the files ready to be written
    path = get_dir_ready(file)

    for i in range(num_problems):
        current_agent_type_count = [0] * int(len(base["robots"]))
        prob = create_new_problem_from_base(base)
        prob_old = convert_prob_to_old(prob)
        write_problem_to_file(prob, path, i, True)
        write_problem_to_file(prob_old, path, i, False)


def get_blank_json_from_base(base):
    new = copy.deepcopy(base)
    new["robots"] = []
    new["tasks"] = []
    new["precedence_constraints"] = []
    new["plan_task_indices"] = []
    return new


def create_new_problem_from_base(base):
    new = get_blank_json_from_base(base)
    new = add_agents_from_base(new, base)
    new = add_task_and_precedence_from_base(new, base)
    return new


def add_task_and_precedence_from_base(new, base):
    new = set_task(new, base)
    new = set_task_locations(new, base)
    return new


def set_task(new, base):
    num_task = random.randrange(num_task_min, num_task_max)
    while len(new["tasks"]) < num_task:
        task_to_add = random.randrange(0, len(base["tasks"]) - 1)
        new["tasks"].append(copy.deepcopy(base["tasks"][task_to_add]))
        new = add_plan_task_indices(new)
        prec_index = new["plan_task_indices"][-1]
        if str(task_to_add) in new["task_with_prec"]:
            for i in range(len(new["task_with_prec"][str(task_to_add)])):
                new["tasks"].append(copy.deepcopy(base["tasks"][new["task_with_prec"][str(task_to_add)][i]]))
                new = add_plan_task_indices(new)
                new["precedence_constraints"].append([prec_index, new["plan_task_indices"][-1]])
    return new


def set_task_locations(new, base):
    new = set_task_init_location(new, base)
    new = set_task_fin_location(new, base)
    return new


def add_plan_task_indices(new):
    new["plan_task_indices"].append(len(new["plan_task_indices"]))
    return new


def add_agents_from_base(new, base):
    new = add_min_agent(new, base)
    additional_agents = random.randrange(num_agent_min - sum(num_type_robot_min),
                                         num_agent_max - sum(num_type_robot_min))
    for i in range(additional_agents):
        new = add_agent_from_base(new, base)
    new = set_starting_locations_agents(new, base)
    return new


def add_min_agent(new, base):
    for i in range(len(base["robots"]) - 1):
        for j in range(num_type_robot_min[i]):
            new["robots"].append(copy.deepcopy(base["robots"][i]))
            new["robots"][-1]["name"] = new["robots"][-1]["name"] + str(current_agent_type_count[i])
            current_agent_type_count[i] = current_agent_type_count[i] + 1
    return new


def add_agent_from_base(new, base):
    need_new_agent_type = True
    while need_new_agent_type:
        agent_type_to_add = random.randrange(0, len(base["robots"]) - 1)
        if current_agent_type_count[agent_type_to_add] < num_type_robot_max[agent_type_to_add]:
            need_new_agent_type = False
    new["robots"].append(copy.deepcopy(base["robots"][agent_type_to_add]))
    new["robots"][-1]["name"] = new["robots"][-1]["name"] + str(current_agent_type_count[agent_type_to_add])
    current_agent_type_count[agent_type_to_add] = current_agent_type_count[agent_type_to_add] + 1
    return new


def set_starting_locations_agents(new, base):
    loc_dic = {}
    for i in range(len(new["species"])):
        loc = sample_agent_location()
        loc_dic[new["species"][i]["name"]] = loc
    for i in range(len(new["robots"])):
        loc = sample_agent_location()
        new["robots"][i]["initial_configuration"]["x"] = loc_dic[new["robots"][i]["species"]][0]
        new["robots"][i]["initial_configuration"]["y"] = loc_dic[new["robots"][i]["species"]][1]
        new["robots"][i]["initial_configuration"]["yaw"] = loc_dic[new["robots"][i]["species"]][2]
    return new


def set_task_init_location(new, base):
    for i in range(len(new["tasks"])):
        loc = sample_task_location()
        new["tasks"][i]["initial_configuration"]["x"] = loc[0]
        new["tasks"][i]["initial_configuration"]["y"] = loc[1]
        new["tasks"][i]["initial_configuration"]["yaw"] = loc[2]
    return new


def set_task_fin_location(new, base):
    for i in range(len(new["tasks"])):
        if (new["action_finish"][new["tasks"][i]["name"]] == "none"):
            new["tasks"][i]["terminal_configuration"]["x"] = new["tasks"][i]["initial_configuration"]["x"]
            new["tasks"][i]["terminal_configuration"]["y"] = new["tasks"][i]["initial_configuration"]["y"]
            new["tasks"][i]["terminal_configuration"]["yaw"] = new["tasks"][i]["initial_configuration"]["yaw"]
        else:
            for j in range(len(new["robots"])):
                if (new["robots"][j]["name"] == new["action_finish"][new["tasks"][i]["name"]]):
                    new["tasks"][i]["terminal_configuration"]["x"] = new["robots"][j]["initial_configuration"]["x"]
                    new["tasks"][i]["terminal_configuration"]["y"] = new["robots"][j]["initial_configuration"]["y"]
                    new["tasks"][i]["terminal_configuration"]["yaw"] = new["robots"][j]["initial_configuration"]["yaw"]

    return new


def sample_agent_location():
    im = cv.imread(agent_loc_pgm + ".pgm", -1)
    row = random.randrange(0, im.shape[0])
    col = random.randrange(0, im.shape[1])
    while im[row][col] != 255:
        row = random.randrange(0, im.shape[0])
        col = random.randrange(0, im.shape[1])
    with open(agent_loc_pgm + ".yaml") as file:
        yaml_config = yaml.load(file, Loader=yaml.FullLoader)
    y = (im.shape[0] - row) * yaml_config["resolution"] + yaml_config["origin"][0]
    x = col * yaml_config["resolution"] + yaml_config["origin"][1]

    return [x, y, 0.0]


def sample_task_location():
    im = cv.imread(task_loc_pgm + ".pgm", -1)
    row = random.randrange(0, im.shape[0])
    col = random.randrange(0, im.shape[1])
    while im[row][col] != 255:
        row = random.randrange(0, im.shape[0])
        col = random.randrange(0, im.shape[1])
    with open(task_loc_pgm + ".yaml") as file:
        yaml_config = yaml.load(file, Loader=yaml.FullLoader)
    y = (im.shape[0] - row) * yaml_config["resolution"] + yaml_config["origin"][0]
    x = col * yaml_config["resolution"] + yaml_config["origin"][1]

    return [x, y, 0.0]


def load_base_old():
    with open(base_loc_old, "r") as read_file:
        base = json.load(read_file)
    return base


def add_agents_from_base_old(prob_old, prob):
    for i in range(len(prob["robots"])):
        prob_old["robots"].append(prob["robots"][i]["name"])
    return prob_old


def add_traits_from_base_old(prob_old, prob):
    species = {}
    for i in prob["species"]:
        species[i["name"]] = i["traits"]
    for i in range(len(prob["robots"])):
        prob_old["robot_traits"].append(species[prob["robots"][i]["species"]])
    return prob_old


def add_task_from_base_old(prob_old, prob):
    for i in range(len(prob["tasks"])):
        prob_old["tasks"].append(prob["tasks"][i]["name"])
    return prob_old


def add_task_reqs_and_dur_from_base_old(prob_old, prob):
    for i in range(len(prob["tasks"])):
        prob_old["tasks_trait_requirements"].append(prob["tasks"][i]["desired_traits"])
        prob_old["tasks_durations"].append(prob["tasks"][i]["duration"])
    return prob_old


def add_precedence_from_base_old(prob_old, prob):
    prob_old["ordering_constraints"] = prob["precedence_constraints"]
    return prob_old


def add_robot_start_from_base_old(prob_old, prob):
    for i in range(len(prob_old["robots"])):
        if prob_old["robots"][i][0] == "A":
            prob_old["robot_start_locations"].append(92)
        if prob_old["robots"][i][0] == "F":
            prob_old["robot_start_locations"].append(93)
        if prob_old["robots"][i][0] == "C":
            prob_old["robot_start_locations"].append(93)
    return prob_old


def add_task_start_end_from_base_old(prob_old, prob):
    for i in range(len(prob_old["tasks"])):
        if prob_old["tasks"][i][0] == "r":
            loc = random.randint(0, 92)
            prob_old["tasks_start_end"].append([loc, loc])
        if prob_old["tasks"][i][0] == "c":
            loc = random.randint(0, 92)
            prob_old["tasks_start_end"].append([loc, loc])

        if prob_old["tasks"][i][0] == "e":
            loc = random.randint(0, 92)
            loc2 = random.randint(0, 92)
            prob_old["tasks_start_end"].append([loc, loc2])
        if prob_old["tasks"][i][0] == "m":
            loc = random.randint(0, 92)
            loc2 = random.randint(0, 92)
            prob_old["tasks_start_end"].append([loc, loc2])
    return prob_old


def convert_prob_to_old(prob):
    prob_old = load_base_old()
    prob_old = add_agents_from_base_old(prob_old, prob)
    prob_old = add_traits_from_base_old(prob_old, prob)
    prob_old = add_task_from_base_old(prob_old, prob)
    prob_old = add_task_reqs_and_dur_from_base_old(prob_old, prob)
    prob_old = add_precedence_from_base_old(prob_old, prob)

    prob_old = add_robot_start_from_base_old(prob_old, prob)
    prob_old = add_task_start_end_from_base_old(prob_old, prob)

    return prob_old


def main():
    # Generate the problems
    write_many_problem_from_base(num_probs)


if __name__ == "__main__":
    main()
