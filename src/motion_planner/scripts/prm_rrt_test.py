#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import random
import time

def dummy_prm(start, goal):
    # Simulated PRM path: A simple straight line with 10 points
    path = []
    for i in range(11):
        x = start[0] + (goal[0] - start[0]) * i / 10
        y = start[1] + (goal[1] - start[1]) * i / 10
        path.append((x, y))
    return path

def publish_path(path_points):
    pub = rospy.Publisher('/planned_path', Path, queue_size=10)
    path_msg = Path()
    path_msg.header.frame_id = "map"
    path_msg.header.stamp = rospy.Time.now()

    for x, y in path_points:
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.w = 1.0
        path_msg.poses.append(pose)

    rospy.sleep(1.0)  # Let publisher register
    pub.publish(path_msg)

def main():
    rospy.init_node('prm_rrt_test')

    # Define simple start and goal
    start = (1.0, 1.0)
    goal = (8.0, 8.0)

    # Time the planning
    t_start = time.time()
    path = dummy_prm(start, goal)
    t_end = time.time()

    rospy.loginfo(f"PRM Path Planning took {t_end - t_start:.3f} seconds")
    rospy.loginfo(f"Path has {len(path)} points")
    
    # Fake collision counter
    collisions = random.randint(0, 3)
    rospy.loginfo(f"Simulated Collisions: {collisions}")

    publish_path(path)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
