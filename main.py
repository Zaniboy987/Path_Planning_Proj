import time
import numpy as np
from robot_simulation import RobotSim
from prm import sample_points, build_roadmap, PRM
from rrt import RRT

def run_test(method="PRM"):
    sim = RobotSim(gui=True)

    start = np.array([0, 0, 0.2])
    goal = np.array([5, 5, 0.2])

    sim.spawn_robot(start)

    # Random obstacles
    for _ in range(10):
        pos = np.random.uniform([0, 0, 0.2], [5, 5, 0.2])
        sim.spawn_obstacle(pos)

    t0 = time.time()

    # **************** TESTING OVER HERE ****************
    if method == "PRM":
        samples = sample_points(200, np.array([[0, 0, 0.2], [5, 5, 0.2]]))
        samples = np.vstack([start, samples, goal])
        edges = build_roadmap(samples, radius=1.5)
        path = PRM(samples, edges, 0, len(samples)-1)
    else:  # RRT
        path = RRT(start, goal, np.array([[0, 0, 0.2], [5, 5, 0.2]]))

    t1 = time.time()
    plan_time = t1 - t0

    # Move Robot
    sim.move_robot(path)

    # Output logs
    with open("log.txt", "a") as f:
        f.write(f"Method: {method}, Time: {plan_time:.4f} sec, Collisions: {sim.collision_count}\n")

    sim.disconnect()

if __name__ == "__main__":
    # You can switch between "PRM" and "RRT"
    for i in range(10):
        run_test("PRM")
    for i in range(10):
        run_test("RRT")
