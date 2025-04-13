n, k = map(int, input().split())
distance_ = {i:[] for i in range(2 * n + 1)}
for i in range(2 * n + 1):
    distance_[i] = list(map(int, input().split()))
memo = {}
min_ = float('inf')
best_path = []

def CBUS(current_city, distance_traveled=0, traveled_path=[0], destination_=set([i for i in range(1, n + 1)]), seat_left=k):
    global min_, best_path

    if len(destination_) == 0:
        distance_traveled = distance_traveled + distance_[current_city][0]
        if distance_traveled < min_:
            min_ = distance_traveled
            best_path = traveled_path
        return
    
    if (current_city, frozenset(traveled_path)) in memo and memo[(current_city, frozenset(traveled_path))] <= distance_traveled:
        return
    
    memo[(current_city, frozenset(traveled_path))] = distance_traveled

    for next_city in destination_:
        if distance_traveled + distance_[current_city][next_city] >= min_:
            continue

        if seat_left > 0:   
            if next_city <= n:
                CBUS(next_city, distance_traveled + distance_[current_city][next_city], traveled_path + [next_city], (destination_ | {next_city + n}) - {next_city}, seat_left - 1)
            else:
                CBUS(next_city, distance_traveled + distance_[current_city][next_city], traveled_path + [next_city], destination_ - {next_city}, seat_left + 1)
        else:
            if next_city <= n:
                continue
            CBUS(next_city, distance_traveled + distance_[current_city][next_city], traveled_path + [next_city], destination_ - {next_city}, seat_left + 1)

if __name__ == '__main__':
    CBUS(0)
    print(min_)
    # print(best_path + [0])
