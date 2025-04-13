import sys
from functools import lru_cache

sys.setrecursionlimit(10000)

n = int(input())  # number of cities
distance = [list(map(int, input().split())) for _ in range(n)]

@lru_cache(None)
def tsp(current, visited):
    if visited == (1 << n) - 1:  # all cities visited
        return distance[current][0]  # return to start

    min_cost = float('inf')
    for next_city in range(n):
        if not (visited >> next_city) & 1:  # if not visited
            new_cost = distance[current][next_city] + tsp(next_city, visited | (1 << next_city))
            min_cost = min(min_cost, new_cost)
    return min_cost

# Start at city 0, only city 0 is visited
print(tsp(0, 1 << 0))

# Solution for getting best path
'''
import sys
from functools import lru_cache

sys.setrecursionlimit(10000)

n = int(input())  # number of cities
distance = [list(map(int, input().split())) for _ in range(n)]

# Cache to store the minimum cost of reaching a state (current city, visited cities)
@lru_cache(None)
def tsp(current, visited):
    if visited == (1 << n) - 1:  # all cities visited
        return distance[current][0]  # return to start

    min_cost = float('inf')
    next_city = -1
    for city in range(n):
        if not (visited >> city) & 1:  # if not visited
            new_cost = distance[current][city] + tsp(city, visited | (1 << city))
            if new_cost < min_cost:
                min_cost = new_cost
                next_city = city  # Track the city that gives the minimum cost

    parent[current, visited] = next_city  # Save the city to reconstruct the path
    return min_cost

# To reconstruct the path, we need to keep track of which city led to the best cost
parent = {}

# Start at city 0, only city 0 is visited
min_cost = tsp(0, 1 << 0)

# Reconstruct the path
path = [0]
visited = 1 << 0
current = 0
while visited != (1 << n) - 1:
    next_city = parent[current, visited]
    path.append(next_city)
    visited |= (1 << next_city)
    current = next_city

# Print the best path and its cost
print("Best path:", *(path + [0]))
print("Minimum cost:", min_cost)
'''