from ortools.linear_solver import pywraplp
from collections import defaultdict, deque

# Input parse
num_tasks, Q_precedence = map(int, input().split())
## Precedence control with in degree and graph for topo sort
in_deg = [0] * num_tasks
graph = [[] for _ in range(num_tasks)]
for _ in range(Q_precedence):
    u, v = map(lambda x: int(x) - 1, input().split())
    graph[u].append(v)
    in_deg[v] += 1

duration = list(map(int, input().split()))
num_teams = int(input())
available = list(map(int, input().split()))

K = int(input())
cost_mat = [[0 for _ in range(num_teams)] for _ in range(num_tasks)]
can_do = [[0 for _ in range(num_teams)] for _ in range(num_tasks)]
for i in range(K):
    task, team, cost = map(int, input().split())
    cost_mat[task - 1][team - 1] = cost
    can_do[task - 1][team - 1] = 1

# Create solver
solver = pywraplp.Solver.CreateSolver("CBC")
bigM = int(1e9)
T = sum(duration) + max(available)

# Decision variables
assignment = [[solver.IntVar(0, 1, "") for j in range(num_teams)] for i in range(num_tasks)]
start_time = [solver.IntVar(0, T, "") for i in range(num_tasks)]
scheduled = [solver.IntVar(0, 1, "") for i in range(num_tasks)]

# Constraints
for i in range(num_tasks):
    solver.Add(sum(assignment[i][j] for j in range(num_teams)) == scheduled[i])

for i in range(num_tasks):
    for j in range(num_teams):
        solver.Add(assignment[i][j] <= can_do[i][j])

for i in range(num_tasks):
    for j in range(num_teams):
        solver.Add(start_time[i] >= assignment[i][j] * available[j])

for i in range(num_tasks):
    if not graph[i]:
        continue
    for j in graph[i]:
        solver.Add(start_time[j] >= start_time[i] + duration[i] * (bigM * (1 - scheduled[i]) + scheduled[i]))

for i in range(num_tasks):
    for j in range(num_tasks):
        for k in range(num_teams):
            b = solver.IntVar(0, 1, "")
            bnot = solver.IntVar(0, 1, "")

            solver.Add(start_time[i] + duration[i] <= start_time[j] + bigM * (2 - assignment[i][k] - assignment[j][k]) + bigM * b)
            solver.Add(start_time[j] + duration[j] <= start_time[i] + bigM * (2 - assignment[i][k] - assignment[j][k]) + bigM * bnot)
            solver.Add((b + bnot) == 1)

# Objectives
num_scheduled = sum(scheduled)
completion_time = solver.IntVar(0, T, "")
for i in range(num_tasks):
    solver.Add(completion_time >= (start_time[i] + duration[i]))
total_cost = sum(cost_mat[i][j] * assignment[i][j] for i in range(num_tasks) for j in range(num_teams))

solver.Maximize(num_scheduled * bigM * bigM - completion_time * bigM - total_cost)

status = solver.Solve()
if status == pywraplp.Solver.OPTIMAL or status == pywraplp.Solver.FEASIBLE:
    print(int(round(num_scheduled.solution_value(), 0)))
    for i in range(num_tasks):
        if scheduled[i].solution_value():
            for j in range(num_teams):
                if assignment[i][j].solution_value():
                    print(i + 1, j + 1, int(round(start_time[i].solution_value(), 0)))