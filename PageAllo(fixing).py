from pprint import pprint
B = list(map(int, input().split()))
S = int(input())
dp = [[None for j in range(S + 1)] for i in range(len(B) + 1)]
C = [[0 for j in range(S + 1)] for i in range(len(B) + 1)]

# availability check
if len(B) < S:
    print(-1)
    exit()

def solve(books, num_stu):
    # base cases
    if num_stu == 1:
        return sum(books)
    elif len(books) == num_stu:
        return max(books)
    
    # available check
    if C[len(books)][num_stu]:
        return dp[len(books)][num_stu]
    
    # main
    res = float('inf')
    for i in range(num_stu - 1, len(books)):
        lhalf = books[:i]
        rhalf = books[i:]
        res = min(res, max(sum(rhalf), solve(lhalf, num_stu-1)))
    dp[len(books)][num_stu] = res
    C[len(books)][num_stu] = 1
    return res

print(solve(B, S))
pprint(dp)