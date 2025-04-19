#!/usr/bin/env python3

import rospy
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from std_msgs.msg import Header
from dstar import DStar
import numpy as np
from visualization_msgs.msg import Marker 


class DStarNode:
    def __init__(self):
        rospy.init_node('dstar_node')

        self.map_data = None
        self.resolution = None
        self.origin = None
        self.width = None
        self.height = None
        self.obstacles = set()

        self.s_start = None
        self.s_goal = None

        rospy.Subscriber("/map", OccupancyGrid, self.map_callback)
        rospy.Subscriber("/initialpose", PoseWithCovarianceStamped, self.start_callback)
        rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.goal_callback)
        
        self.path_pub = rospy.Publisher("/dstar_path", Path, queue_size=10)          # nav_msgs/Path

        rospy.loginfo("D* ROS Node initialized")

    def map_callback(self, msg):
        self.map_data = msg
        self.resolution = msg.info.resolution
        self.origin = (msg.info.origin.position.x, msg.info.origin.position.y)
        self.width = msg.info.width
        self.height = msg.info.height
        print(self.width)
        # Convert OccupancyGrid to obstacle set
        data = np.array(msg.data).reshape((self.height, self.width))
        for y in range(self.height):
            for x in range(self.width):
                if data[y][x] > 50:  # threshold for obstacle
                    self.obstacles.add((x, self.height - y - 1))  # flip y-axis

    def world_to_map(self, x, y):
        mx = int((x - self.origin[0]) / self.resolution)
        my = int((y - self.origin[1]) / self.resolution)
        return (mx, my)

    def map_to_world(self, mx, my):
        x = mx * self.resolution + self.origin[0]
        y = my * self.resolution + self.origin[1]
        return (x, y)

    def start_callback(self, msg):
        self.s_start = self.world_to_map(msg.pose.pose.position.x, msg.pose.pose.position.y)
        rospy.loginfo(f"Start set to {self.s_start}")
        self.try_plan()

    def goal_callback(self, msg):
        self.s_goal = self.world_to_map(msg.pose.position.x, msg.pose.position.y)
        rospy.loginfo(f"Goal set to {self.s_goal}")
        self.try_plan()

    def try_plan(self):
        if self.s_start and self.s_goal and self.map_data:
            dstar = DStar(self.s_start, self.s_goal, self.width, self.height, self.obstacles)
            dstar.run(self.s_start, self.s_goal)
            self.publish_path(dstar.path)

    def publish_path(self, path):
        ros_path = Path()
        ros_path.header = Header()
        ros_path.header.stamp = rospy.Time.now()
        ros_path.header.frame_id = "map"

        for mx, my in path:
            wx, wy = self.map_to_world(mx, my)

            # Path 메시지용
            pose = PoseStamped()
            pose.pose.position.x = wx
            pose.pose.position.y = wy
            pose.pose.orientation.w = 1.0
            ros_path.poses.append(pose)

        self.path_pub.publish(ros_path)


if __name__ == '__main__':
    try:
        DStarNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
