from ortools.linear_solver import pywraplp
from ortools.sat.python import cp_model

# Setup input
# Num teachers, num classes
m, n = list(map(int, input().split()))
# Preference matrix
preference = [[0 for i in range(n)] for j in range(m)]
for i in range(m):
    content = list(map(lambda x: int(x) - 1, input().split()))
    num, courses = content[0] + 1, content[1:]
    for course in courses:
        preference[i][course] = 1
# Number of conflicting classes
k = int(input())
# Conflicting list
conflict = []
for i in range(k):
    conflict.append(tuple(map(lambda x: int(x) - 1, input().split())))

def solution1():    # Solution using cp_model
    # Call model, solver
    model = cp_model.CpModel()
    solver = cp_model.CpSolver()

    # Model variables
    T = [[model.NewBoolVar(f'Teacher {i} teach class {j}') for j in range(n)] for i in range(m)]
    max_load = model.NewIntVar(0, n + 1, f"Maximum number of class a teacher teach")

    # Constraint
    for (i, j) in conflict:
        for teacher in range(m):
            model.add(T[teacher][i] + T[teacher][j] <= 1)   # A teacher can only teach 1 of the conflicting classes
    
    for i in range(m):
        for j in range(n):
            if preference[i][j] == 0:
                model.add(T[i][j] == 0)     # A teacher can only teach class of his preference
        model.add(sum(T[i]) <= max_load)    # Defining max_load

    model.add(sum(T[i][j] for i in range(m) for j in range(n)) == n)    # All n classes must be assigned
    for j in range(n):
        model.add(sum(T[i][j] for i in range(m)) == 1)  # 1 class only assigned to 1 teacher
    # Objective
    model.Minimize(max_load)

    # Solve
    status = solver.solve(model)
    if status == cp_model.OPTIMAL:
        print(int(solver.ObjectiveValue()))
    else:
        print(-1)

def solution2():
    # Solver
    solver = pywraplp.Solver.CreateSolver("SCIP")
    if not solver:
        return
    # Variables
    T = [[solver.BoolVar(f'Teacher {i} teach class {j}') for j in range(n)] for i in range(m)]
    max_load = solver.IntVar(0, n, f"Maximum number of class a teacher teach")
    # Constraint
    for i in range(m):
        for j in range(n):
            if preference[i][j] == 0:
                solver.Add(T[i][j] == 0)     # A teacher can only teach class of his preference
        solver.Add(sum(T[i]) <= max_load)    # Defining max_load
    
    for j in range(n):
        solver.Add(sum(T[i][j] for i in range(m)) == 1)  # 1 class only assigned to 1 teacher

    for (i, j) in conflict:
        for teacher in range(m):
            solver.Add(T[teacher][i] + T[teacher][j] <= 1)   # A teacher can only teach 1 of the conflicting classes

    solver.Add(sum(T[i][j] for i in range(m) for j in range(n)) == n)
    # Objective
    solver.Minimize(max_load)

    # Solve
    status = solver.Solve()
    if status == pywraplp.Solver.OPTIMAL:
        print(int(solver.Objective().Value()))
    else:
        print(-1)

if __name__ == "__main__":
    solution2()