import numpy as np

n, m = list(map(int, input().split()))
C = list(map(float, input().split()))
C = 0 - np.array(C + [0 for i in range(m + 1)])
A = [None for j in range(m)]

for i in range(m):
    A[i] = list(map(float, input().split()))
    ones = [0 for i in range(m)]
    ones[i] = 1
    A[i] = A[i] + ones
b = list(map(float, input().split()))
for i in range(m):
    A[i].append(b[i])
    A[i] = np.array(A[i])

X = [-1 for i in range(n)]

def solve(n, m, C, A, X):
    while min(C[:m + n]) < 0:
        idx = np.argmin(C[:m + n])
        _min, _pivot, bounded = float('inf'), -1, False
        for i in range(m):
            if A[i][idx] != 0:
                if (A[i][-1] / A[i][idx] > 0) and (A[i][-1] / A[i][idx] < _min):
                    _min, _pivot, bounded = A[i][-1] / A[i][idx], i, True

        if bounded == False:
            print("UNBOUNDED")
            return

        A[_pivot] = (1/A[_pivot][idx]) * A[_pivot]
        C = C - C[idx] * A[_pivot]
        for i in range(m):
            if i != _pivot:
                A[i] = A[i] - A[_pivot] * A[i][idx]

    for i in range(n):
        if C[i] != 0:
            X[i] = 0.0
        else:
            for j in range(m):
                if A[j][i] == 1:
                    X[i] = A[j][-1]
		
    print(n)
    for i in range(n):
        print(X[i], end=' ')

if __name__ == "__main__":
    solve(n, m, C, A, X)