import random
random.seed(1)
def N_queen(N, max_steps=100000):
    queens = list(range(N))
    random.shuffle(queens)
 
    col_count = [0] * N
    diag1_count = [0] * (2 * N)
    diag2_count = [0] * (2 * N)    
 
    for r in range(N):
        c = queens[r]
        col_count[c] += 1
        diag1_count[r - c + N] += 1
        diag2_count[r + c] += 1
 
    for _ in range(max_steps):
        conflicted_rows = []
        for r in range(N):
            c = queens[r]
            if col_count[c] + diag1_count[r - c + N] + diag2_count[r + c] > 3:
                conflicted_rows.append(r)
 
        if not conflicted_rows:
            return [col + 1 for col in queens]
 
        r = random.choice(conflicted_rows)
        old_c = queens[r]
 
        min_conf = N + 1
        best_cols = []
 
        for c in range(N):
            conf = col_count[c] + diag1_count[r - c + N] + diag2_count[r + c]
            if c == old_c:
                conf -= 3  
            if conf < min_conf:
                min_conf = conf
                best_cols = [c]
            elif conf == min_conf:
                best_cols.append(c)
 
        new_c = random.choice(best_cols)
 
        if new_c != old_c:
            col_count[old_c] -= 1
            diag1_count[r - old_c + N] -= 1
            diag2_count[r + old_c] -= 1
 
            queens[r] = new_c
 
            col_count[new_c] += 1
            diag1_count[r - new_c + N] += 1
            diag2_count[r + new_c] += 1
 
    return None  
N = int(input())
solution = N_queen(N)
 
if solution:
    print(N)
    print(" ".join(map(str, solution[:])))