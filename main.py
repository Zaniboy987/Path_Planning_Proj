import time
import numpy as np
from robot_simulation import RobotSim
from prm import sample_points, build_roadmap, PRM
from rrt import RRT

def run_test(method="PRM"):
    while True:  # Keep retrying until a valid path is found
        sim = RobotSim(gui=True)

        start = np.array([0, 0, 0.2])
        goal = np.array([5, 5, 0.2])

        sim.spawn_robot(start)

        # Random obstacles
        """for _ in range(10):
            pos = np.random.uniform([0, 0, 0.2], [5, 5, 0.2])
            sim.spawn_obstacle(pos)"""
        # Spawn 3 moving obstacles
        moving_obstacle_positions = [np.random.uniform([0, 0, 0.2], [5, 5, 0.2]) for _ in range(3)]
        for pos in moving_obstacle_positions:
            sim.spawn_obstacle(pos)

        # Assign random velocities to obstacles
        obstacle_velocities = [np.random.uniform([-0.1, -0.1, 0], [0.1, 0.1, 0]) for _ in range(3)]


        t0 = time.time()

        try:
            # **************** TESTING OVER HERE ****************
            if method == "PRM":
                samples = sample_points(300, np.array([[0, 0, 0.2], [5, 5, 0.2]]))
                samples = np.vstack([start, samples, goal])
                edges = build_roadmap(samples, radius=1.5, sim=sim)
                path = PRM(samples, edges, 0, len(samples) - 1)
                if len(path) == 0:  # Empty path — planning failed
                    print("PRM planning failed — retrying.")
                    sim.disconnect()
                    continue
            else:  # RRT
                path = RRT(start, goal, np.array([[0, 0, 0.2], [5, 5, 0.2]]), sim)

            t1 = time.time()
            plan_time = t1 - t0

            # Move Robot
            path_distance = sim.compute_path_distance(path)
            sim.move_robot(path, obstacle_velocities, sim_context=sim, goal=goal, method=method)

            # Output logs
            with open("log.txt", "a") as f:
                f.write(f"Method: {method}, Time: {plan_time:.4f} sec, Distance: {path_distance:.4f}, Collisions: {sim.collision_count}\n")

            sim.disconnect()
            break  # Exit loop after successful run

        except Exception as e:
            print(f"Error encountered: {e}. Retrying...")
            sim.disconnect()
            continue

if __name__ == "__main__":
    # You can switch between "PRM" and "RRT"
    for i in range(1):
        run_test("RRT")
    for i in range(1):
        run_test("PRM")
