import pybullet as p
import pybullet_data
import numpy as np
import time

class RobotSim:
    def __init__(self, gui=True):
        self.gui = gui
        if gui:
            p.connect(p.GUI)
        else:
            p.connect(p.DIRECT)

        p.setGravity(0, 0, -9.8)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        self.plane = p.loadURDF("plane.urdf")
        self.robot = None
        self.obstacles = []
        self.collision_count = 0

    def spawn_robot(self, position):
        self.robot = p.loadURDF("turtlebot3_description/urdf/turtlebot3_burger.urdf", position)

    def spawn_obstacle(self, position, size=(0.5, 0.5, 0.5)):
        visual_shape_id = p.createVisualShape(shapeType=p.GEOM_BOX, halfExtents=size)
        collision_shape_id = p.createCollisionShape(shapeType=p.GEOM_BOX, halfExtents=size)
        obstacle_id = p.createMultiBody(baseMass=0,
                                        baseCollisionShapeIndex=collision_shape_id,
                                        baseVisualShapeIndex=visual_shape_id,
                                        basePosition=position)
        self.obstacles.append(obstacle_id)

    def compute_path_distance(self, path):
        distance = 0.0
        for i in range(1, len(path)):
            distance += np.linalg.norm(np.array(path[i]) - np.array(path[i - 1]))
        return distance

    def move_obstacle(self, obstacle_id, new_position):
        p.resetBasePositionAndOrientation(obstacle_id, new_position, [0, 0, 0, 1])

    def update_moving_obstacles(self, obstacle_velocities):
        for i, obs_id in enumerate(self.obstacles):
            pos, _ = p.getBasePositionAndOrientation(obs_id)
            new_pos = [pos[j] + obstacle_velocities[i][j] for j in range(3)]
            new_pos = [max(0, min(5, new_pos[0])), max(0, min(5, new_pos[1])), pos[2]]
            self.move_obstacle(obs_id, new_pos)

    def is_collision_free(self, p1, p2, step_size=0.05):
        dist = np.linalg.norm(p2 - p1)
        steps = max(1, int(dist / step_size))

        for i in range(steps + 1):
            interp = p1 + (p2 - p1) * (i / steps)
            p.resetBasePositionAndOrientation(self.robot, interp, [0, 0, 0, 1])
            p.stepSimulation()
            collisions_this_step = 0
            for obs in self.obstacles:
                if p.getContactPoints(self.robot, obs):
                    collisions_this_step += 1
            if collisions_this_step > 0:
                return False
        return True

    def move_robot(self, path, obstacle_velocities=None, sim_context=None, goal=None, method="RRT"):
        for i, waypoint in enumerate(path):
            p.resetBasePositionAndOrientation(self.robot, waypoint, [0, 0, 0, 1])
            self.check_collisions()
            if obstacle_velocities:
                self.update_moving_obstacles(obstacle_velocities)
            p.stepSimulation()
            #time.sleep(0.5)

            if self.collision_count > 0:
                print(f"Collision occurred at step {i + 1}. Total collisions so far: {self.collision_count}")


    def check_collisions(self):
        for obs in self.obstacles:
            if p.getContactPoints(self.robot, obs):
                self.collision_count += 1

    def disconnect(self):
        p.disconnect()
