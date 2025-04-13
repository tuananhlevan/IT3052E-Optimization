#PYTHON 
from ortools.linear_solver import pywraplp

A, n, m, alpha = map(int, input().split())
alpha /= 100
invest_category = list(map(lambda x: int(x) - 1, input().split()))
max_invest = list(map(float, input().split()))
risk_prob = list(map(lambda x: float(x) / 100, input().split()))
profit_prob = list(map(lambda x: float(x) / 100, input().split()))
category_max_invest = list(map(float, input().split()))

def solve():
    solver = pywraplp.Solver.CreateSolver("GLOP")
    if not solver:
        return

    X = []
    for i in range(n):
        X.append(solver.NumVar(0, max_invest[i], f'x{i}'))
    solver.Add(sum(X) <= A)
    for i in range(m):
        temp = 0.0
        for j in range(n):
            if invest_category[j] == i:
                temp += X[j]
        solver.Add(temp <= category_max_invest[i])

    solver.Add(sum([X[i] * (risk_prob[i] - alpha) for i in range(n)]) <= 0)

    solver.Maximize(sum([profit_prob[i] * X[i] for i in range(n)]))

    status = solver.Solve()
    if status == pywraplp.Solver.OPTIMAL:
        print(f'{solver.Objective().Value():.1f}')
        # print([f'{X[i].solution_value():.1f}' for i in range(n)])
    else:
    	print(-1)

solve()
