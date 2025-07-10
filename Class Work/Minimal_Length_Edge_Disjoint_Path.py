from ortools.sat.python import cp_model

n_node, m_edge = map(int, input().split())
target = n_node - 1

dist_mat = [[0 for i in range(n_node)] for i in range(n_node)]
can_go = [[0 for i in range(n_node)] for i in range(n_node)]
for i in range(m_edge):
    u, v, w = map(int, input().split())
    dist_mat[u - 1][v - 1] = w
    can_go[u - 1][v - 1] = 1

model = cp_model.CpModel()
solver = cp_model.CpSolver()

mat1 = [[model.NewBoolVar(f'{i}1_{j}1') for j in range(n_node)] for i in range(n_node)]
mat2 = [[model.NewBoolVar(f'{i}2_{j}2') for j in range(n_node)] for i in range(n_node)]

for i in range(n_node):
    for j in range(n_node):
        model.Add(mat1[i][j] + mat2[i][j] <= 1)
        if not can_go[i][j]:
            model.Add(mat1[i][j] == 0)
            model.Add(mat2[i][j] == 0)

for v in range(n_node):
    if v == 0:
        model.Add(sum(mat1[v][i] for i in range(n_node)) == 1)
        model.Add(sum(mat1[i][v] for i in range(n_node)) == 0)

        model.Add(sum(mat2[v][i] for i in range(n_node)) == 1)
        model.Add(sum(mat2[i][v] for i in range(n_node)) == 0)

    elif v == target:
        model.Add(sum(mat1[v][i] for i in range(n_node)) == 0)
        model.Add(sum(mat1[i][v] for i in range(n_node)) == 1)

        model.Add(sum(mat2[v][i] for i in range(n_node)) == 0)
        model.Add(sum(mat2[i][v] for i in range(n_node)) == 1)

    else:
        model.Add(sum(mat1[i][v] for i in range(n_node)) == sum(mat1[v][j] for j in range(n_node)))
        model.Add(sum(mat2[i][v] for i in range(n_node)) == sum(mat2[v][j] for j in range(n_node)))

obj = []
for i in range(n_node):
    for j in range(n_node):
        if can_go[i][j]:
            obj.append(mat1[i][j] * dist_mat[i][j])
            obj.append(mat2[i][j] * dist_mat[i][j])

model.Minimize(sum(obj))

status = solver.Solve(model)
if status == cp_model.FEASIBLE or status == cp_model.OPTIMAL:
    print(int(solver.ObjectiveValue()))
else:
    print("NOT_FEASIBLE")