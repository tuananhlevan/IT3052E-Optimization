import random
from collections import defaultdict, deque
import time
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

class chromosome():
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
    for task in order:
        best_choice = None
        best_time = float('inf')
        best_cost = float('inf')

        for team in cost[task].keys():
            # Earliest start time: max(team_available, all precessor end time)
            earliest = max(teamAvailable[team], max([taskEnd[pred] for pred in range(num_task) if task in graph[pred]] or [0]))
            cost_ = cost[task][team]
            # Choose greedy: best time -> best cost
            if (earliest < best_time) or (earliest == best_time and cost_ < best_cost):
                best_time = earliest
                best_cost = cost_
                best_choice = team
        if best_choice != None:
            task_order.append(task)
            team_assignment.append(best_choice)
            taskStart[task] = best_time
            taskEnd[task] = best_time + duration[task]
            teamAvailable[best_choice] = best_time + duration[task]
    return chromosome(task_order, team_assignment)

def construct_chromosome(num_task, precedence, available_team, can_do):
    task_order = random_topological_sort(num_task, precedence, can_do)
    team_assignment = [random.choice(available_team[i]) for i in task_order]
    individual = chromosome(task_order, team_assignment)
    return individual

def initialize_population(population_size, num_task, precedence, available_team, team_available, can_do, cost, duration):
    population = []
    ind = greedy_construct(num_task, precedence, precedence_graph, team_available, duration, cost)
    population.append(ind)
    for i in range(population_size - 1):
        ind = construct_chromosome(num_task, precedence, available_team, can_do)
        population.append(ind)
    return population

def decode_chromosome(chromosome, duration, num_task, team_available, precedence_graph):
    schedule = []
    taskStart = [None] * num_task
    taskEnd = [0] * num_task
    teamAvailable = team_available[:]

    task_order, team_assignment = chromosome.task_order, chromosome.team_assignment
    queue_length = len(task_order)
    for idx in range(queue_length):
        currentTask, team = task_order[idx], team_assignment[idx]
        earliest = max(teamAvailable[team], max([taskEnd[pred] for pred in range(num_task) if currentTask in precedence_graph[pred]] or [0]))
        teamAvailable[team] = earliest + duration[currentTask]
        taskStart[currentTask] = earliest
        taskEnd[currentTask] = earliest + duration[currentTask]
        schedule.append([currentTask, team, earliest])
    return schedule

def calc_fitness(chromosome, num_task, cost, duration, team_available, precedence_graph):
    schedule = decode_chromosome(chromosome, duration, num_task, team_available, precedence_graph)
    numTask, completion, totalCost = 0, 0, 0
    for task, team, time in schedule:
        numTask += 1
        completion = max(completion, time + duration[task])
        totalCost += cost[task][team]
    return (num_task - numTask, completion, totalCost)

def rank_selection(population, duration, team_available, precedence_graph):
    population = sorted(population, key= lambda x: calc_fitness(x, num_task, cost, duration, team_available, precedence_graph))
    k = len(population)
    probs = [0] * k
    parents = []
    sp = 1.5
    for i in range(k):
        probs[i] = 1 / k * (sp - (2 * sp - 2) * i / (k - 1))
    for i in range(2, k):
        probs[i] += probs[i - 1]
    
    for i in range(2):
        r = random.random()
        for i in range(k):
            if probs[i] >= r:
                parents.append(population[i])
                break
    parents.extend([population[-2], population[-1]])
    return parents[0], parents[1]

def is_precedence_feasible(task_order, precedence):
    pos = {task: idx for idx, task in enumerate(task_order)}
    for (i, j) in precedence:
        if i in pos and j in pos:
            if pos[i] >= pos[j]:
                return False
    return True

def POX(parent1, parent2, precedence):
    child = [-1] * len(parent1)
    added = set()
    for i in range(len(parent1)):
        r = random.random()
        if r >= 0.5:
            child[i] = parent1[i]
            added.add(parent1[i])
    j = 0
    for i in range(len(child)):
        if child[i] == -1:
            while parent2[j] in added:
                j += 1
            child[i] = parent2[j]
            j += 1
    if not is_precedence_feasible(child, precedence):
        child = topological_sort(num_task, precedence, precedence_graph, child)
    return child

def TAX(parent1, parent2, available_team, task_order):
    child = [-1] * len(parent1)
    for i in range(len(parent1)):
        currentTask = task_order[i]
        possible = [lst[i] for lst in [parent1, parent2] if lst[i] in available_team[currentTask]]
        if possible:
            child[i] = random.choice(possible)
        else:
            child[i] = random.choice(available_team[currentTask])
    return child

def offspring_generate(parent1, parent2, precedence, available_team):
    task_order1, team1 = parent1.task_order, parent1.team_assignment
    task_order2, team2 = parent2.task_order, parent2.team_assignment
    
    child_task1 = POX(task_order1, task_order2, precedence)
    child_task2 = POX(task_order2, task_order1, precedence)

    child_team1 = TAX(team1, team2, available_team, child_task1)
    child_team2 = TAX(team2, team1, available_team, child_task2)

    child1 = chromosome(child_task1, child_team1)
    child2 = chromosome(child_task2, child_team2)
    return child1, child2

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

def mutate(ind, precedence, available_team, attempts=5, mutation_rate=0.05):
    task_order, team_assignment = ind.task_order[:], ind.team_assignment[:]
    new_task_order = task_order_mutation(task_order, team_assignment, precedence, attempts)
    new_team_assignment = team_assignment_mutation(team_assignment, new_task_order, available_team, mutation_rate)
    return chromosome(new_task_order, new_team_assignment)

def evaluate_and_replace(population, child1, child2, cost, duration, team_available, precedence_graph):
    population_size = len(population)
    population.extend([child1, child2])
    population.sort(key=lambda x: calc_fitness(x, num_task, cost, duration, team_available, precedence_graph))
    return population[:population_size]

start = time.time()
max_generations = 500
max_time = 290
max_converge = 50

population = initialize_population(10, num_task, precedence, available_team, s, can_do, cost, task_duration)
for gen in range(1, max_generations + 1):
    current_time = time.time() - start
    if current_time >= max_time:
        break
    parents = rank_selection(population, task_duration, s, precedence_graph)
    child1, child2 = offspring_generate(parents[0], parents[1], precedence, available_team)
    child1 = mutate(child1, precedence, available_team)
    child2 = mutate(child2, precedence, available_team)
    population = evaluate_and_replace(population, child1, child2, cost, task_duration, s, precedence_graph)
    if (gen % max_converge) == 0:
        best_chromosome = population[0]
        best_fitness = calc_fitness(best_chromosome, num_task, cost, task_duration, s, precedence_graph)

best_chromosome = population[0]
best_schedule = decode_chromosome(best_chromosome, task_duration, num_task, s, precedence_graph)
best_schedule = sorted(best_schedule, key=lambda x: x[0])
print(len(best_schedule))
for task, team, start in best_schedule:
    print(task + 1, team + 1, start)