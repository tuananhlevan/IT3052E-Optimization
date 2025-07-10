num_city = int(input())
distance_ = {i: [] for i in range(num_city)}
for i in range(num_city):
    distance_[i] = list(map(int, input().split()))

min_ = float('inf')
best_path = []

# Memoization: store the minimum distance for each partial path
memo = {}

def backtrack(current_city, traveled_path, distance_traveled=0, traversal_set=None):
    global min_, best_path
    if traversal_set is None:
        traversal_set = {i for i in range(1, num_city)}
    
    # If we visit all cities
    if len(traversal_set) == 0:
        total_distance = distance_traveled + distance_[current_city][0]  # Add cost to return to the start
        if total_distance < min_:
            min_ = total_distance
            best_path = traveled_path
        return
    
    # If we've already visited the same set of cities, avoid recomputing
    if (current_city, frozenset(traversal_set)) in memo and memo[(current_city, frozenset(traversal_set))] <= distance_traveled:
        return

    # Store the current path cost in memo
    memo[(current_city, frozenset(traversal_set))] = distance_traveled

    for next_city in traversal_set:
        # Pruning: If we can't get a better solution, skip this path
        if distance_traveled + distance_[current_city][next_city] >= min_:
            continue
        
        # Recursive call to visit the next city
        backtrack(next_city, traveled_path + [next_city], distance_traveled + distance_[current_city][next_city], traversal_set - {next_city})

if __name__ == '__main__':
    backtrack(0, [0])
    print(min_)
    # print(best_path + [0])
