import heapq
import math

class Node():
    def __init__(self, level, path, bound):
        self.level = level  # Current level of this node
        self.path = path    # Current path taken
        self.bound = bound  # Level bound of this node

    def __lt__(self, other):
        return self.bound < other.bound

def calculate_bound(matrix, path):
    n = len(matrix)
    bound = 0

    # Mark visited city
    visited = [False for i in range(n)]
    for i in path:
        visited[i] = True
    
    # Add the cost of edges in current path
    for i in range(1, len(path)):
        bound += matrix[path[i - 1]][path[i]]

    # Add minimum edges for unvisited city
    for i in range(n):
        if not visited[i]:
            min_cost = math.inf
            for j in range(n):
                if not visited[j] and matrix[i][j] > 0:
                    min_cost = min(min_cost, matrix[i][j])
                
            if min_cost != math.inf:
                bound += min_cost
    
    return bound

def tsp_bnb(matrix):
    n = len(matrix)
    priority_queue = []

    # Start with the root node (city 0 as the starting point)
    root = Node(0, [0], calculate_bound(matrix, [0]))
    heapq.heappush(priority_queue, root)

    best_cost = math.inf
    best_path = None

    while priority_queue:
        # Get the node with the smallest bound
        current_node = heapq.heappop(priority_queue)

        # If this node's bound is worse than the current best cost, prune it
        if current_node.bound >= best_cost:
            continue
        
        # If all cities are visited, calculate the total cost of the tour
        if current_node.level == n - 1:
            # Complete the tour by returning to the start city
            last_city = current_node.path[-1]
            total_cost = calculate_bound(matrix, current_node.path) + matrix[last_city][0]

            # Update the best solution if found
            if total_cost < best_cost:
                best_cost = total_cost
                best_path = current_node.path + [0]
        
        else:
            # Expand the current node by visiting unvisited cities
            for next_city in range(n):
                if next_city not in current_node.path:
                    new_path = current_node.path + [next_city]
                    new_bound = calculate_bound(matrix, new_path)

                    # If the best bound is promising, add the node to the queue
                    if new_bound < best_cost:
                        heapq.heappush(priority_queue, Node(current_node.level + 1, new_path, new_bound))
        
    return best_cost, best_path

num_city = int(input())
matrix = [None for i in range(num_city)]
for i in range(num_city):
    matrix[i] = list(map(int, input().split()))

print(tsp_bnb(matrix)[0], f'\n{tsp_bnb(matrix)[1]}')