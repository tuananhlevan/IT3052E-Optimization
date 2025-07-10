import random
from collections import defaultdict, deque
import time
import math
random.seed(42)

num_task, Q_constraint = map(int, input().split())
precedence = []
for _ in range(Q_constraint):
    precedence.append(tuple(map(lambda x: int(x) - 1, input().split())))  # task[0] done then can do task[1]
precedence_graph = {u: [] for u in range(num_task)}
for (u, v) in precedence:
    precedence_graph[u].append(v)

task_duration = list(map(int, input().split()))

num_team = int(input())
s = list(map(int, input().split())) # team i available at point s[i]

K = int(input())
cost = defaultdict(dict)
can_do = [0 for i in range(num_task)]
for _ in range(K):
    i, j, w = map(int, input().split())
    can_do[i - 1] = 1
    cost[i - 1][j - 1] = w # assigning task i to team j cost -> cost[(i - 1, j - 1)]
available_team = [list(cost[i].keys()) for i in range(num_task)]

start = time.time()
class state():
    def __init__(self, task_order, team_assignment):
        self.task_order = task_order
        self.team_assignment = team_assignment

def topological_sort(num_task, precedence, precedence_graph, currentOrder):
    in_deg = [0] * num_task
    for (u, v) in precedence:
        in_deg[v] += 1

    priority = {task: i for i, task in enumerate(currentOrder)}
    line = deque(sorted([i for i in range(num_task) if not in_deg[i]], key=lambda x: priority.get(x, float('inf'))))
    order = []
    while line:
        u = line.popleft()
        order.append(u)
        for v in precedence_graph[u]:
            in_deg[v] -= 1
            if in_deg[v] == 0:
                line.append(v)
        line = deque(sorted(line, key=lambda x: priority.get(x, float('inf'))))
    return [i for i in order if can_do[i]]

def random_topological_sort(num_task, precedence, can_do):
    in_deg = [0] * num_task
    for (u, v) in precedence:
        in_deg[v] += 1

    line = deque([i for i in range(num_task) if not in_deg[i]])
    order = []
    while line:
        random.shuffle(line)
        u = line.popleft()
        order.append(u)
        for v in precedence_graph[u]:
            in_deg[v] -= 1
            if in_deg[v] == 0:
                line.append(v)
    return [i for i in order if can_do[i]]

def greedy_construct(num_task, precedence, graph, team_available, duration, cost):
    in_deg = [0] * num_task
    for (u, v) in precedence:
        in_deg[v] += 1
    queue = deque()
    for task in range(num_task):
        if in_deg[task] == 0:
            queue.append(task)
    order = []
    while queue:
        u = queue.popleft()
        order.append(u)
        for v in graph[u]:
            in_deg[v] -= 1
            if in_deg[v] == 0:
                queue.append(v)

    taskStart = [None] * num_task
    taskEnd = [0] * num_task
    teamAvailable = team_available[:]
    task_order = []
    team_assignment = []
    for currentTask in order:
        best_choice = None
        best_time = float('inf')
        best_cost = float('inf')
        for team in cost[currentTask].keys():
            # Earliest start time: max(team_available, all precessor end time)
            earliest = max(teamAvailable[team], max([taskEnd[pred] for pred in range(num_task) if currentTask in graph[pred]] or [0]))
            cost_ = cost[currentTask][team]
            # Choose greedy: best time -> best cost
            if (earliest < best_time) or (earliest == best_time and cost_ < best_cost):
                best_time = earliest
                best_cost = cost_
                best_choice = team
        if best_choice != None:
            task_order.append(currentTask)
            team_assignment.append(best_choice)
            taskStart[currentTask] = best_time
            taskEnd[currentTask] = best_time + duration[currentTask]
            teamAvailable[best_choice] = best_time + duration[currentTask]
    return state(task_order, team_assignment)

def construct_state(num_task, precedence, available_team, can_do):
    task_order = random_topological_sort(num_task, precedence, can_do)
    team_assignment = [random.choice(available_team[i]) for i in task_order]
    individual = state(task_order, team_assignment)
    return individual

def decode_state(state_, duration, num_task, team_available, precedence_graph):
    schedule = []
    taskStart = [None] * num_task
    taskEnd = [0] * num_task
    teamAvailable = team_available[:]

    task_order, team_assignment = state_.task_order, state_.team_assignment
    queue_length = len(task_order)
    for idx in range(queue_length):
        currentTask, team = task_order[idx], team_assignment[idx]
        earliest = max(teamAvailable[team], max([taskEnd[pred] for pred in range(num_task) if currentTask in precedence_graph[pred]] or [0]))
        teamAvailable[team] = earliest + duration[currentTask]
        taskStart[currentTask] = earliest
        taskEnd[currentTask] = earliest + duration[currentTask]
        schedule.append([currentTask, team, earliest])
    return schedule

def calc_fitness(state_, num_task, cost, duration, team_available, precedence_graph):
    schedule = decode_state(state_, duration, num_task, team_available, precedence_graph)
    numTask, completion, totalCost = 0, 0, 0
    for task, team, time in schedule:
        numTask += 1
        completion = max(completion, time + duration[task])
        totalCost += cost[task][team]
    return (num_task - numTask, completion, totalCost)

def is_precedence_feasible(task_order, precedence):
    pos = {task: idx for idx, task in enumerate(task_order)}
    for (i, j) in precedence:
        if i in pos and j in pos:
            if pos[i] >= pos[j]:
                return False
    return True

def task_order_mutation(task_order, team_assignment, precedence, attempts):
    for k in range(attempts):
        new_order = task_order[:]
        i, j = random.sample(range(len(new_order)), 2)
        new_order[i], new_order[j] = new_order[j], new_order[i]
        if is_precedence_feasible(new_order, precedence):
            team_assignment[i], team_assignment[j] = team_assignment[j], team_assignment[i]
            return new_order
    return task_order

def team_assignment_mutation(team_assignment, task_order, available_team, mutation_rate):
    new_assignment = team_assignment[:]
    for idx, task in enumerate(task_order):
        r = random.random()
        if r >= mutation_rate:
            viable = available_team[task]
            new_assignment[idx] = random.choice(viable)
    return new_assignment

def get_neighbor(ind, precedence, available_team, attempts=5, mutation_rate=0.5):
    task_order, team_assignment = ind.task_order[:], ind.team_assignment[:]
    new_task_order = task_order_mutation(task_order, team_assignment, precedence, attempts)
    new_team_assignment = team_assignment_mutation(team_assignment, new_task_order, available_team, mutation_rate)
    return state(new_task_order, new_team_assignment)

def neighbors_generate(ind, precedence, available_team, num_neighbors=10, attempts=5, mutation_rate=0.5):
    neighbors = []
    for _ in range(num_neighbors):
        neighbor = get_neighbor(ind, precedence, available_team, attempts=5, mutation_rate=0.5)
        neighbors.append(neighbor)
    return neighbors

def move_identify(curr, neighbor):
    old_order, old_team = curr.task_order, curr.team_assignment
    new_order, new_team = neighbor.task_order, neighbor.team_assignment
    order_change = None
    team_changes = []
    for (a, b) in zip(old_order, new_order):
        if a != b:
            order_change = ("swap", a, b)
    for idx, (a, b) in enumerate(zip(old_team, new_team)):
        if a != b:
            team_changes.append(("assign", new_order[idx], a, b))

    if order_change or team_changes:
        return ("composite", order_change, team_changes)
    return ("noop",)

def tabu_check(move, tabu_list):
    if move[0] == "noop":
        return False
    swap, assigns = move[1], move[2]
    swap_list, assign_list = tabu_list["task_swap"], tabu_list["team_reassign"]
    if (swap in swap_list) or any([assign in assign_list for assign in assigns]):
        return False
    return True

max_iter = 10000
max_time = 295
max_converge = 100
tabu_list = {"task_swap": defaultdict(), "team_reassign": defaultdict()}
tabu_tenure = 10
curr = greedy_construct(num_task, precedence, precedence_graph, s, task_duration, cost)
best = curr
f_best = calc_fitness(best, num_task, cost, task_duration, s, precedence_graph)
for gen in range(1, max_iter + 1):
    if time.time() - start >= max_time:
        break
    
    neighbors = neighbors_generate(curr, precedence, available_team)
    best_neighbor = None
    best_neighbor_fit = (float('inf'), 0, 0)
    for neighbor in neighbors:
        move = move_identify(curr, neighbor)
        fit = calc_fitness(neighbor, num_task, cost, task_duration, s, precedence_graph)
        if tabu_check(move, tabu_list) or (fit < f_best):
            if fit < best_neighbor_fit:
                best_neighbor = neighbor
                best_neighbor_fit = fit
                selected_move = move
        
        if best_neighbor is None:
            break
        curr = best_neighbor
        if best_neighbor_fit < f_best:
            best = best_neighbor
            f_best = best_neighbor_fit

        tabu_list["task_swap"][selected_move[1]] = gen + tabu_tenure
        for assign in selected_move[2]:
            tabu_list["team_reassign"][assign] = gen + tabu_tenure
        expired_swap = [m for m, expire in tabu_list["task_swap"].items() if expire <= gen] 
        expired_assign = [m for m, expire in tabu_list["team_reassign"].items() if expire <= gen]
        for (swap, assign) in zip(expired_swap, expired_assign):
            del tabu_list["task_swap"][swap]
            del tabu_list["team_reassign"][assign]

schedule = decode_state(best, task_duration, num_task, s, precedence_graph)
schedule.sort()
print(len(schedule))
for team, task, start in schedule:
    print(team + 1, task + 1, start)