n = int(input())
K = int(input())
diffLeft = [True for i in range(2*n + 1)]
sumLeft = [True for i in range(2*n + 1)]
colsLeft = [True for i in range(n + 1)]
rowsLeft = [True for i in range(n + 1)]
for _ in range(K):
    r, c = map(int, input().split())
    if not colsLeft[c] or not diffLeft[r - c + n] or not sumLeft[r + c]:
        print(0)
        exit()
    rowsLeft[r] = False
    colsLeft[c] = False
    diffLeft[r - c + n] = False
    sumLeft[r + c] = False
    
counts = 0

# Instead of iterating though rows and cols like before, I only iterate through cols,
# and fix current working row
def nQueens(row):
    global counts
    if row == n + 1:
        counts += 1
        return
    
    if rowsLeft[row] == False:
        nQueens(row + 1)
        return
    
    for col in range(1, n + 1):
        if colsLeft[col] and diffLeft[row - col + n] and sumLeft[row + col]:
            colsLeft[col] = diffLeft[row - col + n] = sumLeft[row + col] = False
            nQueens(row + 1)
            colsLeft[col] = diffLeft[row - col + n] = sumLeft[row + col] = True

nQueens(1)
print(counts)