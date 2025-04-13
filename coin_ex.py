n, X = map(int, input().split())
D = set(map(int, input().split()))

dp = [1e10 for _ in range(X + 1)]
check = [0 for _ in range(X + 1)]
for i in D:
    dp[i] = 1
    check[i] = 1

def coin_ex(D, X):
    if X < min(D):
        return float('inf')
    elif check[X]:
        return dp[X]
    else:
        dp[X] = min([1 + coin_ex(D, X-i) for i in D])
        check[X] = 1
        return dp[X]
        
print(coin_ex(D, X))