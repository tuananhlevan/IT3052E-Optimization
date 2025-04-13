from ortools.sat.python import cp_model
from collections import defaultdict

num_tasks, Q_constraint = map(int, input().split())
precedence = []
for _ in range(Q_constraint):
    precedence.append(tuple(map(lambda x: int(x) - 1, input().split())))  # task[0] done then can do task[1]

task_duration = list(map(int, input().split()))

num_teams = int(input())
s = list(map(int, input().split())) # team i available at point s[i]

K = int(input())
costs = [[0 for i in range(num_teams)] for j in range(num_tasks)]
can_do = [[0 for i in range(num_teams)] for j in range(num_tasks)]
for _ in range(K):
    i, j, w = map(int, input().split())
    can_do[i - 1][j - 1] = 1
    costs[i - 1][j - 1] = w # assigning task i to team j cost -> cost[(i - 1, j - 1)]

# Finding cycles and removing precedence related to cycles
def find_cycles(precedence, num_tasks):
    # Step 1: Build the adjacency list
    graph = defaultdict(list)
    for u, v in precedence:
        graph[u].append(v)

    # Step 2: Tarjan's Algorithm for Strongly Connected Components (SCCs)
    index = 0
    stack = []
    indices = [-1] * num_tasks  # Discovery time of nodes
    low_link = [-1] * num_tasks  # Lowest reachable node
    on_stack = [False] * num_tasks
    cycles = []

    def tarjan(v):
        nonlocal index
        indices[v] = low_link[v] = index
        index += 1
        stack.append(v)
        on_stack[v] = True

        for neighbor in graph[v]:
            if indices[neighbor] == -1:
                tarjan(neighbor)
                low_link[v] = min(low_link[v], low_link[neighbor])
            elif on_stack[neighbor]:
                low_link[v] = min(low_link[v], indices[neighbor])

        # If v is the root of an SCC, collect all nodes in this SCC
        if low_link[v] == indices[v]:  
            scc = []
            while stack:
                node = stack.pop()
                on_stack[node] = False
                scc.append(node)
                if node == v:
                    break
            if len(scc) > 1:  # Only consider SCCs with cycles (size > 1)
                cycles.append(scc)

    for i in range(num_tasks):
        if indices[i] == -1:
            tarjan(i)

    return cycles  # Returns a list of sets, each containing a cycle

cycles = find_cycles(precedence, num_tasks) # Getting components in cycles
cycles_components = []
for cycle in cycles:
    cycles_components += cycle
  
affected = set(cycles_components[:])  # Set of all components which preresquiste is in a cycle
new_affected = affected.copy()
while True:
    for k in precedence:
        if k[0] in affected:
            new_affected.add(k[1])
    if new_affected == affected:
        break
    affected = new_affected.copy()
for u in affected:
    for j in range(num_teams):
        can_do[u][j] = 0

# Creating model and solver
model = cp_model.CpModel()
limit = sum(task_duration) + max(s)
solver = cp_model.CpSolver()

# Model variables:
assignment = [[model.NewBoolVar(f'Assigned task_{i} to team_{j}') for i in range(num_teams)] for j in range(num_tasks)] # Indicates if task i is assigned to team j
start = [model.NewIntVar(0, limit, f'Start time to do task_{i}') for i in range(num_tasks)] # Start time of tasks
end = [model.NewIntVar(0, limit, f'End time to do task_{i}') for i in range(num_tasks)] # End time of doing tasks
completion = model.NewIntVar(0, limit, f'Finish time setup') # Completion time overall

# Constraints modelling
for i in range(num_tasks):
    if max(can_do[i]): # Constraint only if able to assign
        model.Add(end[i] == start[i] + task_duration[i]) # End time = Start time + duration
        model.Add(end[i] <= completion) # Completion time = max(End time)
    for j in range(num_teams):
        model.Add(start[i] >= s[j]).OnlyEnforceIf(assignment[i][j]) # Start doing tasks after available only if assigned
        if can_do[i][j] == 0:
            model.Add(assignment[i][j] == 0) # Not assigned if is unable to do task

for (i, j) in precedence:
    if max(can_do[i]) and max(can_do[j]):
        model.Add(start[j] >= end[i]) # Order of doing tasks
for i in range(num_tasks):
    model.Add(sum(assignment[i][j] for j in range(num_teams) if can_do[i][j]) <= 1) # Each task is only assigned at most once

for j in range(num_teams):
    teams_interval = [] # Every tasks interval of a team
    for i in range(num_tasks):
        if can_do[i][j]:
            interval = model.NewOptionalIntervalVar(start[i], task_duration[i], end[i], assignment[i][j], f'interval task {i}') # Interval of team j doing task i if assigned
            teams_interval.append(interval) # Add the above interval to the list
    if teams_interval:
        model.AddNoOverlap(teams_interval) # Intervals is not overlapping

# Objectives optimization
model.Maximize(sum(assignment[i][j] for i in range(num_tasks) for j in range(num_teams) if can_do[i][j])) # Objective 1
status = solver.Solve(model)
best_num_task = int(solver.ObjectiveValue()) # Get objective value 1

model.Add(sum(assignment[i][j] for i in range(num_tasks) for j in range(num_teams)) >= best_num_task) # Add objective value 1 as a constraint
model.Minimize(completion) # Objective 2
solver.Solve(model)
best_completion_time = int(solver.ObjectiveValue()) # Get objective value 2

model.Add(completion <= best_completion_time) # Add objective value 2 as a constraint
model.Minimize(sum(assignment[i][j] * costs[i][j] for i in range(num_tasks) for j in range(num_teams))) # Objective 3
status = solver.Solve(model)

# Print outputs
if status == cp_model.OPTIMAL or status == cp_model.FEASIBLE:
    print(f"Number of tasks assigned: {sum(solver.value(assignment[i][j]) for i in range(num_tasks) for j in range(num_teams))}")
    for i in range(num_tasks):
        for j in range(num_teams):
            if solver.value(assignment[i][j]):
                print(f"Task {i + 1} assigned to team {j + 1}, Start time: {solver.value(start[i])}, End time: {solver.value(end[i])}")
    print(f"Total completion time: {best_completion_time}")