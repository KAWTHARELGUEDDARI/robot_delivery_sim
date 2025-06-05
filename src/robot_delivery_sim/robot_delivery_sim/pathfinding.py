import heapq

def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def a_star(grid, start, goal):
    frontier = [(0, start)]
    came_from = {start: None}
    cost_so_far = {start: 0}

    while frontier:
        _, current = heapq.heappop(frontier)

        if current == goal:
            break

        for dx, dy in [(-1,0), (1,0), (0,-1), (0,1)]:
            next = (current[0] + dx, current[1] + dy)
            if grid.in_bounds(*next) and not grid.is_occupied(*next):
                new_cost = cost_so_far[current] + 1
                if next not in cost_so_far or new_cost < cost_so_far[next]:
                    cost_so_far[next] = new_cost
                    priority = new_cost + heuristic(goal, next)
                    heapq.heappush(frontier, (priority, next))
                    came_from[next] = current

    # reconstruire le chemin
    path = []
    current = goal
    while current != start:
        path.append(current)
        current = came_from.get(current)
        if current is None:
            return []  # aucun chemin
    path.append(start)
    path.reverse()
    return path
