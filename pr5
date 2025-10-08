import heapq

def manhattan_dist(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def neighbors(pos, grid):
    directions = [(0,1),(1,0),(0,-1),(-1,0)]
    result = []
    for d in directions:
        nx, ny = pos[0] + d[0], pos[1] + d[1]
        if 0 <= nx < len(grid) and 0 <= ny < len(grid[0]) and grid[nx][ny] != -1:
            result.append((nx, ny))
    return result

def reconstruct_path(came_from, current):
    path = [current]
    while current in came_from:
        current = came_from[current]
        path.append(current)
    path.reverse()
    return path

def greedy_best_first_search(grid, start, goal):
    open_set = []
    heapq.heappush(open_set, (manhattan_dist(start, goal), start))
    came_from = {}
    visited = set([start])

    while open_set:
        _, current = heapq.heappop(open_set)

        if current == goal:
            return reconstruct_path(came_from, current)

        for neighbor in neighbors(current, grid):
            if neighbor not in visited:
                visited.add(neighbor)
                heapq.heappush(open_set, (manhattan_dist(neighbor, goal), neighbor))
                came_from[neighbor] = current
    return None

def a_star_search(grid, start, goal):
    open_set = []
    g_score = {start: 0}
    f_score = {start: manhattan_dist(start, goal)}
    heapq.heappush(open_set, (f_score[start], start))
    came_from = {}

    while open_set:
        _, current = heapq.heappop(open_set)

        if current == goal:
            return reconstruct_path(came_from, current), g_score[current]

        for neighbor in neighbors(current, grid):
            tentative_g_score = g_score[current] + grid[neighbor[0]][neighbor[1]]
            if tentative_g_score < g_score.get(neighbor, float('inf')):
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = tentative_g_score + manhattan_dist(neighbor, goal)
                heapq.heappush(open_set, (f_score[neighbor], neighbor))

    return None, float('inf')

grid = [
    [1, 1, 1, 1, 1],
    [1, -1, 2, -1, 1],
    [1, 1, 2, 1, 1],
    [1, -1, 1, -1, 1],
    [1, 1, 1, 1, 1]
]

start = (0, 0)
goal = (4, 4)

path_greedy = greedy_best_first_search(grid, start, goal)
path_a_star, cost_a_star = a_star_search(grid, start, goal)

print("Greedy Best-First Search Path:")
print(path_greedy)
print()

print("A* Search Path and Cost:")
print(path_a_star)
print("\nTotal Cost:", cost_a_star)
