n, M = map(int, input().split())
a = list(map(int, input().split()))
count = 0

def solution(a, M):
    global count
    if len(a) == 1:
        count += (M % a[0] == 0)
        return
    
    for i in range(1, M):
        if a[-1] * i >= M:
            break
        solution(a[:-1], M - i * a[-1])

solution(a, M)
print(count)