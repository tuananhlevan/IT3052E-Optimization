fund, numSelection, numSection, totalRate = map(int, input().split())
selections = list(map(lambda x: int(x) - 1, input().split()))
maxSpendSelection = list(map(int, input().split()))
failRate = list(map(int, input().split()))
profitRate = list(map(int, input().split()))
maxSpendSection = list(map(int, input().split()))

from ortools.linear_solver import pywraplp

solver = pywraplp.Solver.CreateSolver("GLOP")

spends = [solver.NumVar(0, maxSpendSelection[i], f"Spending for the selection {i}th") for i in range(numSelection)]

totalFail = sum([spends[i] * (failRate[i] - totalRate) for i in range(numSelection)])
selectionSpends = {i:0 for i in range(numSelection)}
for i in range(numSelection):
    selectionSpends[selections[i]] += spends[i]
totalSpend = sum(spends)

solver.Add(totalFail <= 0)
for i in range(numSection):
    solver.Add(selectionSpends[i] <= maxSpendSection[i])
solver.Add(totalSpend <= fund)

totalProfit = [spends[i] * profitRate[i] for i in range(numSelection)]
solver.Maximize(sum(totalProfit))

status = solver.Solve()
if status == pywraplp.Solver.OPTIMAL or status == pywraplp.Solver.FEASIBLE:
    print(f"{(solver.Objective().Value() / 100):.1f}")
else:
    print(-1)