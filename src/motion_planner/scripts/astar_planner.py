#!/usr/bin/env python3

import rospy
import heapq
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

class Node:
    def __init__(self, x, y, cost=0, parent=None):
        self.x = x
        self.y = y
        self.cost = cost
        self.parent = parent

    def __lt__(self, other):
        return self.cost < other.cost

def heuristic(node, goal):
    return math.hypot(goal.x - node.x, goal.y - node.y)

def a_star(start, goal):
    open_set = []
    heapq.heappush(open_set, (0, start))
    closed_set = set()

    while open_set:
        _, current = heapq.heappop(open_set)
        if (current.x, current.y) == (goal.x, goal.y):
            path = []
            while current:
                path.append((current.x, current.y))
                current = current.parent
            return path[::-1]  # reverse path

        closed_set.add((current.x, current.y))

        for dx, dy in [(-1,0), (1,0), (0,-1), (0,1)]:
            nx, ny = current.x + dx, current.y + dy
            if (nx, ny) in closed_set:
                continue
            neighbor = Node(nx, ny, current.cost + 1, current)
            priority = neighbor.cost + heuristic(neighbor, goal)
            heapq.heappush(open_set, (priority, neighbor))
    return None

def move_robot(path):
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10)

    for x, y in path:
        twist = Twist()
        twist.linear.x = 0.5  # move forward
        twist.angular.z = 0.0
        pub.publish(twist)
        rospy.sleep(1.0)  # crude step timing

    pub.publish(Twist())  # stop

def publish_path(path):
    pub = rospy.Publisher('/planned_path', Path, queue_size=10)
    path_msg = Path()
    path_msg.header.frame_id = "map"
    path_msg.header.stamp = rospy.Time.now()

    for x, y in path:
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.w = 1.0
        path_msg.poses.append(pose)

    rospy.sleep(1.0)
    pub.publish(path_msg)

def main():
    rospy.init_node('astar_planner')
    start_node = Node(0, 0)
    goal_node = Node(20, 5)

    rospy.loginfo("Running A* path planner...")
    path = a_star(start_node, goal_node)

    if path:
        rospy.loginfo("Path found, moving Jackal...")
        publish_path(path)
        move_robot(path)
    else:
        rospy.logerr("No path found!")

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
