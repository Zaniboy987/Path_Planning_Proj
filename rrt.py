import numpy as np
import pybullet as p

class Node:
    def __init__(self, position, parent=None):
        self.position = position
        self.parent = parent

def euclidean(p1, p2):
    return np.linalg.norm(p1 - p2)

def is_collision_free(sim, p1, p2, step_size=0.05):
    dist = np.linalg.norm(p2 - p1)
    steps = max(1, int(dist / step_size))  # Ensure at least one step

    for i in range(steps + 1):
        interp = p1 + (p2 - p1) * (i / steps)
        p.resetBasePositionAndOrientation(sim.robot, interp, [0, 0, 0, 1])
        p.stepSimulation()
        for obs in sim.obstacles:
            if p.getContactPoints(sim.robot, obs):
                return False
    return True

def RRT(start, goal, bounds, sim, n_iter=250, step_size=0.5):
    nodes = [Node(start)]

    for _ in range(n_iter):
        rand_point = np.random.uniform(bounds[0], bounds[1])
        nearest = min(nodes, key=lambda node: euclidean(node.position, rand_point))
        direction = rand_point - nearest.position
        length = np.linalg.norm(direction)
        if length > 0:
            direction = direction / length

        new_pos = nearest.position + step_size * direction

        # Check for collision before adding new node
        if is_collision_free(sim, nearest.position, new_pos):
            new_node = Node(new_pos, nearest)
            nodes.append(new_node)

            if euclidean(new_pos, goal) < step_size:
                goal_node = Node(goal, new_node)
                nodes.append(goal_node)
                break

    # Reconstruct path
    path = []
    node = nodes[-1]
    while node is not None:
        path.append(node.position)
        node = node.parent
    path.reverse()
    return path
