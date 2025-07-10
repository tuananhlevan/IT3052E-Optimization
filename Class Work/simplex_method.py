class Array(list):
    def __init__(self, data: iter):
        super(Array, self).__init__(data)
        self.data = data
    
    def __add__(self, other):
        if isinstance(other, (int, float)):
            return Array([x + other for x in self.data])
        elif isinstance(other, Array):
            return Array([x + y for x, y in zip(self.data, other.data)])
        
    def __radd__(self, other):
        return self.__add__(other)
    
    def __sub__(self, other):
        if isinstance(other, (int, float)):
            return Array([x - other for x in self.data])
        elif isinstance(other, Array):
            return Array([x - y for x, y in zip(self.data, other.data)])
    
    def __rsub__(self, other):
        if isinstance(other, (int, float)):
            return Array([other - x for x in self.data])
        elif isinstance(other, Array):
            return Array([y - x for x, y in zip(self.data, other.data)])

    def __mul__(self, other):
        if isinstance(other, (int, float)):
            return Array([x * other for x in self.data])
        elif isinstance(other, Array):
            return Array([x * y for x, y in zip(self.data, other.data)])
        
    def __rmul__(self, other):
        return self.__mul__(other)
    
    def __repr__(self):
        return f"Array({self.data})"
    
    def __getitem__(self, idx):
        result = super(Array, self).__getitem__(idx)
        if isinstance(idx, slice):
            return Array(result)
        return result
    
    def argmin(self):
        min_val = min(self.data)
        return self.index(min_val)

n, m = list(map(int, input().split()))
C = list(map(float, input().split()))
C = 0 - Array(C + [0 for i in range(m + 1)])
A = [None for j in range(m)]

for i in range(m):
    A[i] = list(map(float, input().split()))
    ones = [0 for i in range(m)]
    ones[i] = 1
    A[i] = A[i] + ones
b = list(map(float, input().split()))
for i in range(m):
    A[i].append(b[i])
    A[i] = Array(A[i])

X = [-1 for i in range(n)]

def solve(n, m, C, A, X):
    while min(C[:m + n]) < 0:
        idx = C[:m + n].argmin()
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
    return

if __name__ == "__main__":
    solve(n, m, C, A, X)