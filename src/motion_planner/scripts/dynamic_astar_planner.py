#!/usr/bin/env python

import rospy
import numpy as np
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from scipy.interpolate import CubicSpline

class DynamicAStarPlanner:
    def __init__(self):
        rospy.init_node('dynamic_astar_planner')
        
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_callback)
        self.scan_sub = rospy.Subscriber("/scan", LaserScan, self.scan_callback)
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        
        self.current_position = (0.0, 0.0)
        self.grid_size = 20  # 20x20 grid
        self.cell_size = 0.5  # each cell is 0.5 meters
        self.grid = np.zeros((self.grid_size, self.grid_size))
        
        self.goal = (8, 8)  # Example goal (meters, meters) from starting point
        
        self.path = []
        self.rate = rospy.Rate(5)  # 5 Hz update rate

    def odom_callback(self, msg):
        self.current_position = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def scan_callback(self, msg):
        self.update_grid_with_scan(msg)
        self.plan_and_move()

    def update_grid_with_scan(self, scan):
        """ Updates self.grid based on latest laser scan """
        self.grid.fill(0)  # Reset grid
        angle = scan.angle_min
        for r in scan.ranges:
            if scan.range_min < r < scan.range_max:
                x = r * np.cos(angle)
                y = r * np.sin(angle)
                grid_x = int((x + self.grid_size * self.cell_size / 2) / self.cell_size)
                grid_y = int((y + self.grid_size * self.cell_size / 2) / self.cell_size)
                if 0 <= grid_x < self.grid_size and 0 <= grid_y < self.grid_size:
                    self.grid[grid_x, grid_y] = 1  # Mark obstacle
            angle += scan.angle_increment

        self.inflate_obstacles()

    def inflate_obstacles(self):
        """ Inflate obstacles in the grid to ensure safety margin """
        inflation_radius = 1  # In cells, adjust as needed
        inflated_grid = np.copy(self.grid)
        
        for i in range(self.grid_size):
            for j in range(self.grid_size):
                if self.grid[i, j] == 1:
                    for dx in range(-inflation_radius, inflation_radius+1):
                        for dy in range(-inflation_radius, inflation_radius+1):
                            ni, nj = i + dx, j + dy
                            if 0 <= ni < self.grid_size and 0 <= nj < self.grid_size:
                                inflated_grid[ni, nj] = 1
        
        self.grid = inflated_grid  # Apply inflated grid

    def heuristic(self, a, b):
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    def astar(self, start, goal):
        """ A simple A* on the self.grid """
        open_set = {start}
        came_from = {}
        g_score = {start: 0}
        f_score = {start: self.heuristic(start, goal)}
        
        while open_set:
            current = min(open_set, key=lambda o: f_score.get(o, np.inf))
            if current == goal:
                return self.reconstruct_path(came_from, current)
            
            open_set.remove(current)
            for neighbor in self.get_neighbors(current):
                tentative_g = g_score[current] + 1
                if tentative_g < g_score.get(neighbor, np.inf):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score[neighbor] = tentative_g + self.heuristic(neighbor, goal)
                    open_set.add(neighbor)
        return []

    def reconstruct_path(self, came_from, current):
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path

    def get_neighbors(self, pos):
        neighbors = []
        directions = [(1,0), (0,1), (-1,0), (0,-1)]  # 4-connectivity
        for d in directions:
            neighbor = (pos[0] + d[0], pos[1] + d[1])
            if 0 <= neighbor[0] < self.grid_size and 0 <= neighbor[1] < self.grid_size:
                if self.grid[neighbor[0], neighbor[1]] == 0:
                    neighbors.append(neighbor)
        return neighbors

    def smooth_path(self, path):
        if len(path) < 3:
            return path
        path_x, path_y = zip(*path)
        cs_x = CubicSpline(range(len(path_x)), path_x)
        cs_y = CubicSpline(range(len(path_y)), path_y)
        smooth_points = 50
        smooth_x = cs_x(np.linspace(0, len(path_x)-1, smooth_points))
        smooth_y = cs_y(np.linspace(0, len(path_y)-1, smooth_points))
        return list(zip(smooth_x, smooth_y))

    def plan_and_move(self):
        # Convert robot position to grid coordinates
        start_x = int((self.current_position[0] + self.grid_size * self.cell_size / 2) / self.cell_size)
        start_y = int((self.current_position[1] + self.grid_size * self.cell_size / 2) / self.cell_size)
        
        goal_x = int((self.goal[0] + self.grid_size * self.cell_size / 2) / self.cell_size)
        goal_y = int((self.goal[1] + self.grid_size * self.cell_size / 2) / self.cell_size)
        
        path = self.astar((start_x, start_y), (goal_x, goal_y))
        if not path:
            rospy.logwarn("No path found!")
            return
        
        smooth_path = self.smooth_path(path)
        self.follow_path(smooth_path)

    def follow_path(self, path):
        """ Simple pure pursuit control """
        if not path:
            return
        
        target = path[0]
        tx = (target[0] - self.grid_size/2) * self.cell_size
        ty = (target[1] - self.grid_size/2) * self.cell_size
        
        dx = tx - self.current_position[0]
        dy = ty - self.current_position[1]
        distance = np.hypot(dx, dy)
        
        if distance < 0.2:
            path.pop(0)
            return
        
        angle_to_target = np.arctan2(dy, dx)
        
        twist = Twist()
        twist.linear.x = 0.5
        twist.angular.z = angle_to_target
        self.cmd_pub.publish(twist)

    def run(self):
        rospy.spin()

if __name__ == "__main__":
    planner = DynamicAStarPlanner()
    planner.run()
