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

team_available = available[:]
task_start = [None] * num_tasks
task_end = [0] * num_tasks

# Topo sort to control constraints
queue = set()
for task in range(num_tasks):
    if in_deg[task] == 0:
        queue.add(task)

best_numTask = -1
best_time = float('inf')
best_cost = float('inf')
best_schedule = None
def calculate_score(schedule):
    schedule = [schedule[i] for i in range(num_tasks) if schedule[i]]
    countTask = len(schedule)
    completionTime = max([schedule[i][2] + duration[schedule[i][0]] for i in range(countTask)])
    totalCost = sum([cost_mat[schedule[i][0]][schedule[i][1]] for i in range(countTask)])

    return countTask, completionTime, totalCost

scheduled = [[]] * num_tasks
def assign(currentOrder, taskAvailable, teamAvailable, taskStart, taskEnd, schedule):
    global cost_mat, num_tasks, best_numTask, best_time, best_cost, duration, best_schedule
    if any(schedule):
        numTask, currentTime, currentCost = calculate_score(schedule)

        if (currentTime > best_time) or (currentTime == best_time and currentCost > best_cost):
            return

    if not taskAvailable:
        if (numTask > best_numTask) or (numTask == best_numTask and currentTime < best_time) or (numTask == best_numTask and currentTime == best_time and currentCost < best_cost):
            print(numTask, currentTime, currentCost)
            best_schedule = schedule[:]
            best_numTask = numTask
            best_time = currentTime
            best_cost = currentCost
        return

    for task in taskAvailable:
        for team in cost_mat[task].keys():
            time = max(team_available[team], max([task_end[pred] for pred in range(num_tasks) if task in graph[pred]] or [0]))

            tempSchedule = schedule[currentOrder][:]
            schedule[currentOrder] = [task, team, time]

            nextAvailable = set()
            for proc in graph[task]:
                in_deg[proc] -= 1
                if in_deg[proc] == 0:
                    nextAvailable.add(proc)

            temp_teamAvailable = teamAvailable[team]
            teamAvailable[team] = time + duration[task]

            taskStart[task] = time
            taskEnd[task] = time + duration[task]
            assign(currentOrder + 1, (taskAvailable | nextAvailable) - {task}, teamAvailable, taskStart, taskEnd, schedule)

            schedule[currentOrder] = tempSchedule
            teamAvailable[team] = temp_teamAvailable
            taskStart[task] = None
            taskEnd[task] = 0
            for proc in graph[task]:
                in_deg[proc] += 1
start = time.time()
assign(0, queue, team_available, task_start, task_end, scheduled)
end = time.time() - start
countTask = 0
countTime = 0
countCost = 0
for task, team, start in best_schedule:
    countTask += 1
    countTime = max(countTime, start + duration[task])
    countCost += cost_mat[task][team]
print(f"Backtracking")
print(f"Final solution: {(countTask, countTime, countCost)}")
print(f"Finish time: {end}")
