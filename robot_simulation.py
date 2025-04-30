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
        self.robot = p.loadURDF("turtlebot3/turtlebot3_description/urdf/turtlebot3_burger.urdf", position)
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

    def move_robot(self, path):
        for waypoint in path:
            p.resetBasePositionAndOrientation(self.robot, waypoint, [0, 0, 0, 1])
            self.check_collisions()
            p.stepSimulation()
            time.sleep(0.05)

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