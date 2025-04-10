from ortools.sat.python import cp_model
from collections import defaultdict, deque
import heapq

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
    graph = defaultdict(list)
    for u, v in precedence:
        graph[u].append(v)

    index = 0
    stack = []
    indices = [-1] * num_tasks
    low_link = [-1] * num_tasks
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

        if low_link[v] == indices[v]:  
            scc = []
            while stack:
                node = stack.pop()
                on_stack[node] = False
                scc.append(node)
                if node == v:
                    break
            if len(scc) > 1:
                cycles.append(scc)

    for i in range(num_tasks):
        if indices[i] == -1:
            tarjan(i)

    return cycles

cycles = find_cycles(precedence, num_tasks)
cycles_components = []
for cycle in cycles:
    cycles_components += cycle

affected = set(cycles_components)
adj = defaultdict(list)
for u, v in precedence:
    adj[u].append(v)

queue = deque(cycles_components)
while queue:
    u = queue.popleft()
    for v in adj[u]:
        if v not in affected:
            affected.add(v)
            queue.append(v)

for u in affected:
    for j in range(num_teams):
        can_do[u][j] = 0

# Optimized Greedy Algorithm
def optimized_greedy_task_assignment(num_tasks, num_teams, task_duration, precedence, s, costs, can_do):
    graph = defaultdict(list)
    indegree = [0] * num_tasks
    for u, v in precedence:
        graph[u].append(v)
        indegree[v] += 1

    task_assignments = [-1] * num_tasks
    task_start_times = [-1] * num_tasks
    task_end_times = [-1] * num_tasks
    team_available_times = s[:]

    ready = []
    for i in range(num_tasks):
        if indegree[i] == 0 and any(can_do[i]):
            heapq.heappush(ready, (-task_duration[i], i))  # prioritize longer tasks

    while ready:
        _, task = heapq.heappop(ready)

        teams = [j for j in range(num_teams) if can_do[task][j]]
        if not teams:
            continue

        earliest_start = 0
        for u, v in precedence:
            if v == task:
                earliest_start = max(earliest_start, task_end_times[u])

        best_team = min(teams, key=lambda j: (max(team_available_times[j], earliest_start), costs[task][j]))
        start_time = max(team_available_times[best_team], earliest_start)
        end_time = start_time + task_duration[task]

        task_assignments[task] = best_team
        task_start_times[task] = start_time
        task_end_times[task] = end_time
        team_available_times[best_team] = end_time

        for v in graph[task]:
            indegree[v] -= 1
            if indegree[v] == 0 and any(can_do[v]):
                heapq.heappush(ready, (-task_duration[v], v))

    return task_assignments, task_start_times, task_end_times

def ortools_solution():
    model = cp_model.CpModel()
    limit = sum(task_duration) + max(s)
    solver = cp_model.CpSolver()
    solver.parameters.num_search_workers = 8

    assignment = [[model.NewBoolVar(f'Assigned task_{i} to team_{j}') for i in range(num_teams)] for j in range(num_tasks)]
    start = [model.NewIntVar(0, limit, f'Start time to do task_{i}') for i in range(num_tasks)]
    end = [model.NewIntVar(0, limit, f'End time to do task_{i}') for i in range(num_tasks)]
    completion = model.NewIntVar(0, limit, f'Finish time setup')

    for i in range(num_tasks):
        if max(can_do[i]):
            model.Add(end[i] == start[i] + task_duration[i])
            model.Add(end[i] <= completion)
        for j in range(num_teams):
            model.Add(start[i] >= s[j]).OnlyEnforceIf(assignment[i][j])
            if can_do[i][j] == 0:
                model.Add(assignment[i][j] == 0)

    for (i, j) in precedence:
        if max(can_do[i]) and max(can_do[j]):
            model.Add(start[j] >= end[i])
    for i in range(num_tasks):
        model.Add(sum(assignment[i][j] for j in range(num_teams) if can_do[i][j]) <= 1)

    for j in range(num_teams):
        teams_interval = []
        for i in range(num_tasks):
            if can_do[i][j]:
                interval = model.NewOptionalIntervalVar(start[i], task_duration[i], end[i], assignment[i][j], f'interval task {i}')
                teams_interval.append(interval)
        if teams_interval:
            model.AddNoOverlap(teams_interval)

    model.Maximize(sum(assignment[i][j] for i in range(num_tasks) for j in range(num_teams) if can_do[i][j]))
    status = solver.Solve(model)
    best_num_task = int(solver.ObjectiveValue())

    model.Add(sum(assignment[i][j] for i in range(num_tasks) for j in range(num_teams)) >= best_num_task)
    model.Minimize(completion)
    solver.Solve(model)
    best_completion_time = int(solver.ObjectiveValue())

    model.Add(completion <= best_completion_time)
    model.Minimize(sum(assignment[i][j] * costs[i][j] for i in range(num_tasks) for j in range(num_teams)))
    status = solver.Solve(model)
        
    if status == cp_model.OPTIMAL or status == cp_model.FEASIBLE:
        print(sum(solver.value(assignment[i][j]) for i in range(num_tasks) for j in range(num_teams)))
        for i in range(num_tasks):
            for j in range(num_teams):
                if solver.value(assignment[i][j]):
                    print(i + 1, j + 1, solver.value(start[i]))
        print(best_completion_time)

# Run the greedy or OR-Tools solution based on scale
if __name__ == "__main__":
    if num_tasks > 200:
        task_assignments, task_start_times, task_end_times = optimized_greedy_task_assignment(
            num_tasks, num_teams, task_duration, precedence, s, costs, can_do
        )

        print(f"{sum(1 for x in task_assignments if x != -1)}")
        for task in range(num_tasks):
            if task_assignments[task] != -1:
                print(f"{task + 1} {task_assignments[task] + 1} {task_start_times[task]}")
    else:
        ortools_solution()
