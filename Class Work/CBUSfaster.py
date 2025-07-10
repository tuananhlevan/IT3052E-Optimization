import sys
sys.setrecursionlimit(10000)
from functools import lru_cache

n, k = map(int, input().split())
distance_ = [list(map(int, input().split())) for _ in range(2 * n + 1)]

FULL = (1 << (2 * n + 1)) - 1  # bitmask for all cities (excluding 0)

@lru_cache(None)
def CBUS(current_city, visited, seat_left):
    if visited == (1 << (2 * n + 1)) - (1 << (2 * n + 1 - 2 * n - 1)):  # All cities visited
        return distance_[current_city][0]  # return to depot

    min_cost = float('inf')

    for next_city in range(1, 2 * n + 1):
        if not (visited >> next_city) & 1:
            if next_city <= n and seat_left > 0:  # pickup
                cost = distance_[current_city][next_city] + CBUS(next_city, visited | (1 << next_city), seat_left - 1)
                if cost < min_cost:
                    min_cost = min(min_cost, cost)
                    best_city = next_city
            elif next_city > n and (visited >> (next_city - n)) & 1:  # delivery only if corresponding pickup is done
                cost = distance_[current_city][next_city] + CBUS(next_city, visited | (1 << next_city), seat_left + 1)
                if cost < min_cost:
                    min_cost = min(min_cost, cost)
                    best_city = next_city

    next_stop[current_city, visited] = best_city
    return min_cost

if __name__ == '__main__':
    next_stop = {}

    best_cost = CBUS(0, 1 << 0, k)  # Start at city 0, mark it as visited
    
    path = [0]
    current = 0
    visited = 1 << 0
    while visited != FULL:
        next_ = next_stop[current, visited]
        path.append(next_)
        visited |= (1 << next_)
        current = next_

    print(best_cost)
    print(*list(map(lambda x: x + 1, path + [0])))