#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
from f110_msgs.msg import Wpnt, WpntArray
import numpy as np

class OdomToFrenetPublisher:
    def __init__(self):
        rospy.init_node("odom_to_frenet_publisher", anonymous=True)
        rospy.on_shutdown(self.on_shutdown)

        # Initialize variables
        self.waypoints_x = None
        self.waypoints_y = None

        # Subscribers
        rospy.Subscriber("/global_path/optimal_trajectory_wpnt", WpntArray, self.global_waypoints_cb)
        rospy.Subscriber("/odom", Odometry, self.odom_cb)

        # Publisher
        self.odom_frenet_pub = rospy.Publisher("/odom_frenet", Odometry, queue_size=10)

        rospy.loginfo("OdomToFrenetPublisher initialized")

    def global_waypoints_cb(self, msg):
        """Callback to receive global waypoints and store them"""
        if self.waypoints_x is None or self.waypoints_y is None:
            self.waypoints_x = np.array([wpnt.x_m for wpnt in msg.wpnts])
            self.waypoints_y = np.array([wpnt.y_m for wpnt in msg.wpnts])
            rospy.loginfo("Waypoints received and stored")

    def odom_cb(self, msg):
        """Callback to compute Frenet coordinates directly and publish"""
        if self.waypoints_x is None or self.waypoints_y is None:
            rospy.logwarn("Waypoints not initialized yet")
            return

        # Extract x, y from Odometry
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        # Compute the closest point on the trajectory
        distances = np.sqrt((self.waypoints_x - x)**2 + (self.waypoints_y - y)**2)
        closest_idx = np.argmin(distances)

        # Calculate s and d directly
        closest_x = self.waypoints_x[closest_idx]
        closest_y = self.waypoints_y[closest_idx]
        s = np.sum(np.sqrt(np.diff(self.waypoints_x[:closest_idx+1])**2 + np.diff(self.waypoints_y[:closest_idx+1])**2))

        tangent_x = self.waypoints_x[min(closest_idx + 1, len(self.waypoints_x) - 1)] - closest_x
        tangent_y = self.waypoints_y[min(closest_idx + 1, len(self.waypoints_y) - 1)] - closest_y
        tangent_norm = np.sqrt(tangent_x**2 + tangent_y**2)

        normal_x = -tangent_y / tangent_norm
        normal_y = tangent_x / tangent_norm

        d = (x - closest_x) * normal_x + (y - closest_y) * normal_y

        # Create new Odometry message for Frenet coordinates
        frenet_msg = Odometry()
        frenet_msg.header = msg.header  # Use the same header as the input message
        frenet_msg.pose.pose.position.x = s  # Set s-coordinate
        frenet_msg.pose.pose.position.y = d  # Set d-coordinate

        # Publish the Frenet coordinates
        self.odom_frenet_pub.publish(frenet_msg)

    def on_shutdown(self):
        rospy.loginfo("Shutting down OdomToFrenetPublisher")

    def loop(self):
        rospy.spin()

if __name__ == "__main__":
    try:
        converter = OdomToFrenetPublisher()
        converter.loop()
    except rospy.ROSInterruptException:
        pass

