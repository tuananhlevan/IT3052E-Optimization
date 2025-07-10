from collections import defaultdict, deque
import time

# Input parse
num_tasks, Q_precedence = map(int, input().split())
## Precedence control with in degree and graph for topo sort
in_deg = [0] * num_tasks
graph = [[] for _ in range(num_tasks)] # graph[u]: all the tasks that has u as precedence
for _ in range(Q_precedence):
    u, v = map(lambda x: int(x) - 1, input().split())
    graph[u].append(v)
    in_deg[v] += 1

duration = list(map(int, input().split()))
num_teams = int(input())
available = list(map(int, input().split()))

K = int(input())
cost_mat = defaultdict(dict)
for i in range(K):
    task, team, cost = map(int, input().split())
    cost_mat[task - 1][team - 1] = cost

begin = time.time()
team_available = available[:]
task_start = [None] * num_tasks
task_end = [0] * num_tasks

# Topo sort to control constraints
queue = deque()
for task in range(num_tasks):
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

scheduled = []
total_cost = 0
for task in order:
    best_choice = None
    best_time = float('inf')
    best_cost = float('inf')

    for team in cost_mat[task].keys():
        # Earliest start time: max(team_available, all precessor end time)
        earliest = max(team_available[team], max([task_end[pred] for pred in range(num_tasks) if task in graph[pred]] or [0]))
        end_time = earliest + duration[task]
        cost = cost_mat[task][team]
        # Choose greedy: best time -> best cost
        if (earliest < best_time) or (earliest == best_time and cost < best_cost):
            best_time = earliest
            best_cost = cost
            best_choice = team
    
    if best_choice != None:
        scheduled.append([task, best_choice, best_time])
        task_start[task] = best_time
        task_end[task] = best_time + duration[task]
        team_available[best_choice] = best_time + duration[task]
        total_cost += best_cost

countTask = 0
countTime = 0
countCost = 0
for task, team, start in scheduled:
    countTask += 1
    countTime = max(countTime, start + duration[task])
    countCost += cost_mat[task][team]
print('-------------------------------')
print(f"Greedy Algorithm")
print(f"Best Solution: {(countTask, countTime, countCost)}")
print(f"Finish time: {time.time() - begin}")