import numpy as np
import heapq

def sample_points(n_samples, bounds):
    return np.random.uniform(bounds[0], bounds[1], size=(n_samples, 3))

def euclidean(p1, p2):
    return np.linalg.norm(p1 - p2)

def build_roadmap(samples, radius, sim):
    edges = {}
    for i, p in enumerate(samples):
        edges[i] = []
        for j, q in enumerate(samples):
            if i != j and euclidean(p, q) < radius:
                if sim.is_collision_free(p, q):
                    edges[i].append(j)
    return edges

def PRM(samples, edges, start_idx, goal_idx):
    frontier = [(0, start_idx)]
    came_from = {start_idx: None}
    cost_so_far = {start_idx: 0}

    while frontier:
        _, current = heapq.heappop(frontier)
        if current == goal_idx:
            break
        for neighbor in edges[current]:
            new_cost = cost_so_far[current] + euclidean(samples[current], samples[neighbor])
            if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                cost_so_far[neighbor] = new_cost
                priority = new_cost + euclidean(samples[neighbor], samples[goal_idx])
                heapq.heappush(frontier, (priority, neighbor))
                came_from[neighbor] = current

    path = []
    current = goal_idx
    while current is not None:
        path.append(samples[current])
        current = came_from[current]
    path.reverse()
    return path
