import pybullet as p
import pybullet_data
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
        #self.robot = p.loadURDF("sphere2.urdf", position)

    def spawn_obstacle(self, position, size=(0.5, 0.5, 0.5)):
        visual_shape_id = p.createVisualShape(shapeType=p.GEOM_BOX,
                                               halfExtents=size)
        collision_shape_id = p.createCollisionShape(shapeType=p.GEOM_BOX,
                                                    halfExtents=size)
        obstacle_id = p.createMultiBody(baseMass=0,
                                        baseCollisionShapeIndex=collision_shape_id,
                                        baseVisualShapeIndex=visual_shape_id,
                                        basePosition=position)
        self.obstacles.append(obstacle_id)

    def move_obstacle(self, obstacle_id, new_position):
        p.resetBasePositionAndOrientation(obstacle_id, new_position, [0, 0, 0, 1])

    def update_moving_obstacles(self, obstacle_velocities):
        for i, obs_id in enumerate(self.obstacles):
            pos, _ = p.getBasePositionAndOrientation(obs_id)
            new_pos = [pos[j] + obstacle_velocities[i][j] for j in range(3)]
            # Keep within bounds [0, 5] for x, y
            new_pos = [max(0, min(5, new_pos[0])), max(0, min(5, new_pos[1])), pos[2]]
            self.move_obstacle(obs_id, new_pos)


    def move_robot(self, path, obstacle_velocities=None):
        for waypoint in path:
            p.resetBasePositionAndOrientation(self.robot, waypoint, [0, 0, 0, 1])
            self.check_collisions()
            if obstacle_velocities:
                self.update_moving_obstacles(obstacle_velocities)
            p.stepSimulation()
            time.sleep(0.5)

    def get_obstacle_ids(self):
        return self.obstacle_ids  # returns list of obstacle body IDs

    def get_obstacle_aabb(self, obstacle_id):
        return self.p.getAABB(obstacle_id)


    def check_collisions(self):
        for obs in self.obstacles:
            contact_points = p.getContactPoints(self.robot, obs)
            if len(contact_points) > 0:
                self.collision_count += 1

    def disconnect(self):
        p.disconnect()