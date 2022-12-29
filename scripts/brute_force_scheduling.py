from math import sqrt
from itertools import combinations, permutations, product


def euclidDistance(x1, y1, x2, y2):
    return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2))


class Robot:
    def __init__(self, id, x, y, speed):
        self.id = id
        self.x = x
        self.y = y
        self.speed = speed


class Task:
    def __init__(self, id, x_i, y_i, x_t, y_t, static_duration, coalition):
        self.id = id
        self.x_i = x_i
        self.y_i = y_i
        self.x_t = x_t
        self.y_t = y_t
        self.static_duration = static_duration
        self.coalition = coalition
        self.earliest_start_time = {}
        self.resetTimes()
        self.preceeding_tasks = set()

    def addPreceedingTask(self, preceeding_task):
        self.preceeding_tasks.add(preceeding_task)

    def resetTimes(self):
        for robot in self.coalition:
            self.earliest_start_time[robot.id] = None

    def startTime(self):
        earliest_start_time = -1
        for task in self.preceeding_tasks:
            if not task.finished():
                return None
            earliest_start_time = max(earliest_start_time, task.finishTime())
        if not self.finished():
            return None
        return max(earliest_start_time, max([self.earliest_start_time[k] for k in self.earliest_start_time]))

    def finishTime(self):
        start_time = self.startTime()
        if start_time is None:
            return None
        return start_time + self.duration()

    def finished(self):
        for task in self.preceeding_tasks:
            if not task.finished():
                return False
        return None not in [self.earliest_start_time[k] for k in self.earliest_start_time]

    def duration(self):
        if not hasattr(self, 'dynamic_duration'):
            self.dynamic_duration = 0
            distance = euclidDistance(self.x_i, self.y_i, self.x_t, self.y_t)
            for robot in self.coalition:
                self.dynamic_duration = max(self.dynamic_duration, distance / robot.speed)
        return self.static_duration + self.dynamic_duration


class Schedule:
    def __init__(self, n):
        self.timepoints = []
        for _ in range(n):
            self.timepoints.append((None, None))
        self.makespan = -1


robots = [
    Robot(0, 0, 0, 0.24),
    Robot(1, 1, 0, 0.2),
    Robot(2, 2, 0, 0.24)
]

tasks = [
    Task(0, 0, 1, 0, 1, 1, [robots[0]]),
    Task(1, 1, 1, 1, 2, 2, [robots[1]]),
    Task(2, 2, 1, 2, 4, 1, [robots[0], robots[2]]),
    Task(3, 3, 3, 3, 3, 2, [robots[1]]),
    Task(4, 2.5, 2.5, 1.7, 1.7, 3, [robots[2]]),
    Task(5, 3.68, 3, 3, 2.5, 1.5, [robots[1]]),
    Task(6, 10, 5, 7, 3.5, 0.5, [robots[0]])
]

allocation = [
    [0, 2, 6],
    [1, 3, 5],
    [2, 4]
]

precedence = set([(0, 1), (0, 2), (1, 3), (2, 3), (2, 4), (3, 4), (5, 2), (5, 6)])

previous_set_length = len(precedence)
while True:
    tmp_precedence = set([])
    for constraint in precedence:
        tmp_precedence.add(constraint)
        for constraint2 in precedence:
            if constraint[1] == constraint2[0]:
                tmp_precedence.add((constraint[0], constraint2[1]))
    precedence = tmp_precedence
    if len(precedence) == previous_set_length:
        break
    previous_set_length = len(precedence)
del previous_set_length

for constraint in precedence:
    tasks[constraint[1]].addPreceedingTask(tasks[constraint[0]])

all_orders = []

for i in range(len(allocation)):
    all_orders.append(permutations(allocation[i]))

initial_locations = []
for robot in robots:
    initial_locations.append((robot.x, robot.y))

all_possible_orders = product(*all_orders)

debug = False
best_schedule = None
num_invalid = 0
for order_index, team_orders in enumerate(all_possible_orders):

    # Check if the precedence is violated
    cannot_execute = False
    for robot in robots:
        prev_task = team_orders[robot.id][0]
        for task_index in range(1, len(team_orders[robot.id])):
            if (team_orders[robot.id][task_index], prev_task) in precedence:
                # print(f'\tInvalid ordering - {prev_task} -> {team_orders[robot.id][j]}')
                cannot_execute = True
                num_invalid += 1
                break
            prev_task = team_orders[robot.id][task_index]
        if cannot_execute:
            break
    if cannot_execute:
        del cannot_execute
        continue

    print(f'Team order {order_index + 1 - num_invalid}:')
    for robot in robots:
        print(f'\t{list(team_orders[robot.id])}')

    for task in tasks:
        task.resetTimes()
    schedule = Schedule(len(tasks))

    iteration = 0
    while True:
        if debug:
            print(f'\tIteration: {iteration}')
        if iteration > 1000:
            print(f'\tMore than 1000 iterations - deadlock')
            break
        iteration += 1
        for task in tasks:
            if task.finished():
                if debug:
                    print(f'\t\tTask {task.id} finished')
                continue

            can_execute = True

            # Precedence
            for constraint in precedence:
                if constraint[1] == task.id and not tasks[constraint[0]].finished():
                    if debug:
                        print(f'\t\tCannot execute {task.id} - {constraint[0]}')
                    can_execute = False
                    break
            if not can_execute:
                del can_execute
                continue

            for robot in task.coalition:
                # Already computed
                if task.earliest_start_time[robot.id] is not None:
                    continue

                if debug:
                    print(f'\t\tRobot {robot.id} allocated to task {task.id}')
                order_index = None
                for task_index, task_id in enumerate(team_orders[robot.id]):
                    if task_id == task.id:
                        if debug:
                            print('\t\t\tPrevious tasks completed', end='')
                        order_index = task_index
                        break
                    if not tasks[task_id].finished():
                        can_execute = False
                        if debug:
                            print(f'\t\t\tCannot execute {task.id} - {task_id}')
                        break
                if not can_execute:
                    break

                # Initial Transition
                if order_index == 0:
                    if debug:
                        print(f' - IT')
                    earliest_start = euclidDistance(task.x_i, task.y_i, initial_locations[robot.id][0], initial_locations[robot.id][1]) / robot.speed
                    task.earliest_start_time[robot.id] = earliest_start
                else:
                    if debug:
                        print(f' - T')
                    previous_task_index = team_orders[robot.id][order_index - 1]
                    previous_task = tasks[previous_task_index]
                    earliest_start = schedule.timepoints[previous_task.id][1] + euclidDistance(task.x_i, task.y_i, previous_task.x_t, previous_task.y_t) / robot.speed
                    task.earliest_start_time[robot.id] = earliest_start
                if debug:
                    print(f"\t\t\tStart: {earliest_start}")
                    print(f"\t\t\tFinish: {earliest_start + task.duration()}")

            if not can_execute:
                del can_execute
                continue

            if task.finished():
                print(f"\t\tTask {task.id} added to the schedule ({task.startTime()}, {task.finishTime()})")
                schedule.timepoints[task.id] = (task.startTime(), task.finishTime())
            else:
                print(f"\t\tNot all robots or preceeding tasks ready to start the task")

        all_tasks_finished = True
        for task in tasks:
            if not task.finished():
                all_tasks_finished = False
                break
        if not all_tasks_finished:
            continue

        makespan = -1
        for timepoints in schedule.timepoints:
            makespan = max(makespan, timepoints[1])
        schedule.makespan = makespan
        break
    if schedule.makespan > 0:
        if best_schedule is None:
            best_schedule = schedule
        elif schedule.makespan < best_schedule.makespan:
            best_schedule = schedule
        print(f'\tmakespan: {schedule.makespan}; timepoints: {schedule.timepoints}')

print(f'Best schedule - makespan: {best_schedule.makespan}; timepoints: {best_schedule.timepoints}')